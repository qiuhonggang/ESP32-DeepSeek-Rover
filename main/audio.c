#include "robot.h"
#include "barge_in.h"
#include "dialog_window.h"
#include "nav.h"
#include "esp_wn_models.h"
#include "model_path.h"
#include <math.h>
#include <stdatomic.h>
#include <string.h>
#include "driver/i2s_std.h"
#include "esp_aec.h"
#include "esp_heap_caps.h"
#include "esp_vad.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PLAY_CAPACITY 16384
#define PLAY_START_SAMPLES (SAMPLE_RATE * 560 / 1000)
#define LOCAL_PLAY_START_SAMPLES (SAMPLE_RATE * 320 / 1000)
#define PLAY_REFILL_SAMPLES (SAMPLE_RATE * 640 / 1000)
#define PREROLL 9
#define MIN_CAPTURE_VOICE_MS 96
#define PENDING_VOICE_US 12000000
#define BARGE_PREROLL_LEAD_MS 64
#define BARGE_PREROLL_MS (BARGE_PREROLL_LEAD_MS + BARGE_VOICE_START_MS + BARGE_VOICE_GAPS_MS + BARGE_CHECK_LIMIT_MS)
#define REF_HISTORY 4
#define TEST_TONE_ID UINT32_MAX
#define WAKE_CUE_ID (UINT32_MAX - 1)
#define PLAY_DRAIN_US 130000
#define PLAY_IDLE_US 8000000
#define ECHO_TAIL_US 250000
#define WAKE_DRAIN_US 950000

const wake_option_t *audio_selected_wake(void)
{
    unsigned choice = robot_config.wake_choice;
    return &wake_options[choice < wake_option_count ? choice : 0];
}

static i2s_chan_handle_t rx, tx;
static aec_handle_t *aec;
static vad_handle_t vad;
static const esp_wn_iface_t *wake;
static model_iface_data_t *wake_data;
static int wake_frame, wake_used;
static int16_t wake_pcm[2048] __attribute__((aligned(16)));
static const int16_t wake_silence[MAX_FRAME];
typedef enum { LISTEN_NONE, LISTEN_CONVERSATION, LISTEN_ASR_ONLY } listen_mode_t;
static listen_mode_t force_listen;
static uint32_t interrupted_id;
static int frame;
static bool initialized;
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE reference_lock = portMUX_INITIALIZER_UNLOCKED;
static audio_status_t telemetry;
static atomic_int volume;
static uint32_t turn_id;
static uint32_t next_id;
static bool dialog_active;
static dialog_window_t dialog_window;
static int64_t pending_voice_until;
static unsigned reset_capture;
static struct {
    int16_t pcm[PLAY_CAPACITY];
    size_t read, length, start_samples;
    uint32_t id;
    int64_t last_rx, drain_after;
    unsigned tone_sample;
    bool active, ended, started, isolated, buffering;
} play;
static int16_t references[REF_HISTORY][MAX_FRAME];
static int16_t reference_pending[MAX_FRAME];
static unsigned reference_index, reference_used;
static unsigned preroll_capacity;
static int16_t *preroll;
static int32_t i2s_rx[MAX_FRAME * 2], i2s_tx[MAX_FRAME * 2];
static int16_t mic_l[MAX_FRAME], mic_r[MAX_FRAME];
static int16_t mono[MAX_FRAME] __attribute__((aligned(16)));
static int16_t reference[MAX_FRAME] __attribute__((aligned(16)));
static int16_t clean[MAX_FRAME] __attribute__((aligned(16)));
static int16_t outgoing[MAX_FRAME];

const char *direction_name(direction_t direction)
{
    const char *names[] = {"unknown", "left", "center", "right"};
    return direction >= DIR_UNKNOWN && direction <= DIR_RIGHT ? names[direction] : "unknown";
}

audio_status_t audio_status(void)
{
    taskENTER_CRITICAL(&state_lock);
    audio_status_t copy = telemetry;
    int64_t now = esp_timer_get_time();
    int64_t remaining_us = dialog_window.expires_us - (now > dialog_window.opens_us ? now : dialog_window.opens_us);
    copy.followup_ms = remaining_us > 0 ? (int)((remaining_us + 999) / 1000) : 0;
    taskEXIT_CRITICAL(&state_lock);
    return copy;
}

const char *playback_result_name(playback_result_t result)
{
    const char *names[] = {"none", "complete", "wake_word", "manual", "i2s_error", "timeout",
                           "network_lost", "cloud_error", "voice"};
    return names[result >= PLAYBACK_NONE && result <= PLAYBACK_VOICE ? result : PLAYBACK_NONE];
}

static bool send_event(uplink_type_t type, uint32_t id, direction_t direction);

static void clear_play(playback_result_t result)
{
    if (play.active && play.id != TEST_TONE_ID && play.id != WAKE_CUE_ID) {
        telemetry.playback_result = result;
    }
    play.active = false;
    play.length = 0;
    play.ended = false;
    play.started = false;
    telemetry.speaking = false;
}

static void interrupt_audio(playback_result_t result)
{
    taskENTER_CRITICAL(&state_lock);
    interrupted_id = turn_id;
    force_listen = LISTEN_NONE;
    clear_play(result);
    ++reset_capture;
    turn_id = 0;
    dialog_active = false;
    dialog_window = (dialog_window_t){0};
    pending_voice_until = 0;
    telemetry.listening = false;
    taskEXIT_CRITICAL(&state_lock);
}

void audio_interrupt(void)
{
    interrupt_audio(PLAYBACK_MANUAL);
}

void audio_network_lost(void)
{
    interrupt_audio(PLAYBACK_NETWORK_LOST);
}

bool audio_listen_once(bool asr_only)
{
    if (!network_ready()) return false;
    taskENTER_CRITICAL(&state_lock);
    bool accepted = initialized && !telemetry.listening && !force_listen;
    if (accepted) force_listen = asr_only ? LISTEN_ASR_ONLY : LISTEN_CONVERSATION;
    taskEXIT_CRITICAL(&state_lock);
    return accepted;
}

static uint32_t next_turn_locked(void)
{
    if (++next_id > INT32_MAX) next_id = 1;
    return next_id;
}

uint32_t audio_begin_debug_turn(void)
{
    taskENTER_CRITICAL(&state_lock);
    uint32_t id = 0;
    if (initialized && !telemetry.listening && !force_listen && !play.active) {
        id = turn_id = next_turn_locked();
        dialog_active = false;
        dialog_window = (dialog_window_t){0};
        pending_voice_until = 0;
    }
    taskEXIT_CRITICAL(&state_lock);
    return id;
}

void audio_cancel_turn(uint32_t id)
{
    taskENTER_CRITICAL(&state_lock);
    if (turn_id == id && id != 0) {
        turn_id = 0;
        dialog_active = false;
        dialog_window = (dialog_window_t){0};
        pending_voice_until = 0;
        ++reset_capture;
        telemetry.listening = false;
    }
    if (play.active && play.id == id) clear_play(PLAYBACK_CLOUD_ERROR);
    taskEXIT_CRITICAL(&state_lock);
}

void audio_end_dialog(uint32_t id)
{
    taskENTER_CRITICAL(&state_lock);
    if (id && turn_id == id) {
        dialog_active = false;
        dialog_window = (dialog_window_t){0};
        pending_voice_until = 0;
    }
    taskEXIT_CRITICAL(&state_lock);
}

static void open_dialog_window(int64_t now)
{
    int64_t opens = now + ECHO_TAIL_US;
    dialog_window = (dialog_window_t){.opens_us = opens, .expires_us = opens + DIALOG_WAIT_MS * 1000};
    pending_voice_until = 0;
}

void audio_offer_followup(uint32_t id)
{
    taskENTER_CRITICAL(&state_lock);
    if (id && turn_id == id && dialog_active && !play.active && !telemetry.listening) {
        open_dialog_window(esp_timer_get_time());
    }
    taskEXIT_CRITICAL(&state_lock);
}

bool audio_turn_is_current(uint32_t id)
{
    taskENTER_CRITICAL(&state_lock);
    bool current = id && id == turn_id && !telemetry.listening;
    taskEXIT_CRITICAL(&state_lock);
    return current;
}

static bool begin_play(uint32_t id, bool isolated, size_t start_samples)
{
    taskENTER_CRITICAL(&state_lock);
    bool accepted = initialized && !telemetry.listening && !play.active &&
                    (id == 0 || id == TEST_TONE_ID || id == WAKE_CUE_ID || id == turn_id);
    if (accepted) {
        play.id = id;
        play.read = play.length = 0;
        play.start_samples = start_samples;
        play.tone_sample = 0;
        play.last_rx = esp_timer_get_time();
        play.drain_after = 0;
        play.active = true;
        play.started = false;
        play.isolated = isolated;
        play.ended = id == TEST_TONE_ID;
        play.buffering = id != TEST_TONE_ID;
    }
    taskEXIT_CRITICAL(&state_lock);
    return accepted;
}

bool audio_begin_play(uint32_t id, bool isolated)
{
    return begin_play(id, isolated, PLAY_START_SAMPLES);
}

bool audio_begin_local_play(uint32_t id)
{
    return begin_play(id, false, LOCAL_PLAY_START_SAMPLES);
}

bool audio_test_tone(void)
{
    return audio_begin_play(TEST_TONE_ID, true);
}

void audio_set_volume(int value)
{
    atomic_store(&volume, value);
}

int audio_volume(void)
{
    return atomic_load(&volume);
}

bool audio_push_play(uint32_t id, const void *data, size_t size)
{
    if (!size || size % 2 || size > MAX_WIRE_BYTES) return false;
    taskENTER_CRITICAL(&state_lock);
    size_t count = size / sizeof(int16_t);
    bool accepted = play.active && play.id == id && !play.ended && count <= PLAY_CAPACITY - play.length;
    if (accepted) {
        size_t start = (play.read + play.length) % PLAY_CAPACITY;
        size_t first = PLAY_CAPACITY - start;
        if (first > count) first = count;
        memcpy(play.pcm + start, data, first * 2);
        if (count > first) memcpy(play.pcm, (const uint8_t *)data + first * 2, (count - first) * 2);
        play.length += count;
        play.last_rx = esp_timer_get_time();
    }
    taskEXIT_CRITICAL(&state_lock);
    return accepted;
}

void audio_end_play(uint32_t id)
{
    taskENTER_CRITICAL(&state_lock);
    if (play.active && play.id == id) play.ended = true;
    taskEXIT_CRITICAL(&state_lock);
}

#define CUE_ASSET(n) \
    extern const uint8_t cue_##n##_start[] asm("_binary_wake_" #n "_pcm_start"); \
    extern const uint8_t cue_##n##_end[] asm("_binary_wake_" #n "_pcm_end")
CUE_ASSET(1);
CUE_ASSET(2);
CUE_ASSET(3);
CUE_ASSET(4);
#undef CUE_ASSET

bool audio_wake_feedback(void)
{
    static const struct { const uint8_t *start, *end; } clips[] = {
        {cue_1_start, cue_1_end}, {cue_2_start, cue_2_end},
        {cue_3_start, cue_3_end}, {cue_4_start, cue_4_end},
    };
    taskENTER_CRITICAL(&state_lock);
    unsigned clip = telemetry.wake_cue % (sizeof(clips) / sizeof(clips[0]));
    size_t bytes = clips[clip].end - clips[clip].start;
    bool accepted = initialized && !play.active && !telemetry.listening && bytes <= sizeof(play.pcm);
    if (accepted) {
        memcpy(play.pcm, clips[clip].start, bytes);
        play.id = WAKE_CUE_ID;
        play.read = 0;
        play.length = bytes / sizeof(int16_t);
        play.last_rx = esp_timer_get_time();
        play.drain_after = 0;
        play.active = play.ended = play.isolated = true;
        play.started = play.buffering = false;
        ++telemetry.wake_cue;
    }
    taskEXIT_CRITICAL(&state_lock);
    return accepted;
}

static bool send_event(uplink_type_t type, uint32_t id, direction_t direction)
{
    uplink_t packet = {.type = type, .id = id, .direction = direction};
    return cloud_uplink(&packet);
}

static bool send_pcm(uint32_t id, const int16_t *pcm)
{
    uplink_t packet = {.type = UP_AUDIO, .id = id, .bytes = (uint16_t)(2 * frame)};
    memcpy(packet.pcm, pcm, packet.bytes);
    return cloud_uplink(&packet);
}

static bool tx_complete(i2s_chan_handle_t handle, i2s_event_data_t *event, void *unused)
{
    (void)handle;
    (void)unused;
    const size_t sample_bytes = 2 * sizeof(int32_t);
    if (!event->dma_buf || event->size % sample_bytes) return false;
    const int32_t *samples = event->dma_buf;
    size_t count = event->size / sample_bytes;
    portENTER_CRITICAL_ISR(&reference_lock);
    for (size_t i = 0; i < count; ++i) {
        reference_pending[reference_used++] = (int16_t)(samples[i * 2] >> 16);
        if (reference_used == frame) {
            unsigned index = ++reference_index % REF_HISTORY;
            memcpy(references[index], reference_pending, frame * sizeof(int16_t));
            reference_used = 0;
        }
    }
    portEXIT_CRITICAL_ISR(&reference_lock);
    return false;
}

static uint32_t fill_output(int64_t now, uint32_t pause_id, uint32_t *playback_id,
                            bool *playing, bool *speaking, bool *isolated)
{
    uint32_t completed = TEST_TONE_ID;
    bool drained = false;
    unsigned tone = 0;
    bool tone_active = false;
    uint32_t first_pcm = 0;
    memset(outgoing, 0, frame * sizeof(int16_t));
    taskENTER_CRITICAL(&state_lock);
    // A voice check temporarily outputs silence and keeps the queued PCM intact.
    if (play.active && (!pause_id || play.id != pause_id)) {
        if (play.id != TEST_TONE_ID && !play.ended && play.started &&
            !play.buffering && play.length < (size_t)frame) {
            play.buffering = true;
            ++telemetry.playback_underruns;
        }
        size_t target = play.started ? PLAY_REFILL_SAMPLES : play.start_samples;
        if (play.ended || play.length >= target) play.buffering = false;
        if (play.id == TEST_TONE_ID && play.tone_sample < 8000) {
            tone_active = true;
            tone = play.tone_sample;
            play.tone_sample += frame;
            play.drain_after = now + PLAY_DRAIN_US;
            play.started = true;
        } else if (play.length && !play.buffering) {
            size_t count = play.length < frame ? play.length : frame;
            size_t first = PLAY_CAPACITY - play.read;
            if (first > count) first = count;
            memcpy(outgoing, play.pcm + play.read, first * 2);
            if (count > first) memcpy(outgoing + first, play.pcm, (count - first) * 2);
            play.length -= count;
            play.read = (play.read + count) % PLAY_CAPACITY;
            play.drain_after = now + PLAY_DRAIN_US;
            if (!play.started && play.id != WAKE_CUE_ID) first_pcm = play.id;
            play.started = true;
        } else if ((play.ended && !play.length && now >= play.drain_after) ||
                   now - play.last_rx > PLAY_IDLE_US) {
            completed = play.id;
            drained = play.ended && !play.length && now >= play.drain_after;
            if (drained && dialog_active && !play.isolated && completed && completed == turn_id) {
                open_dialog_window(now);
            }
            clear_play(drained ? PLAYBACK_COMPLETE : PLAYBACK_TIMEOUT);
        }
    }
    *playback_id = play.active ? play.id : 0;
    *playing = play.active;
    *speaking = *playing && play.started;
    *isolated = *playing && play.isolated;
    telemetry.speaking = *speaking;
    taskEXIT_CRITICAL(&state_lock);

    if (completed != TEST_TONE_ID && completed != WAKE_CUE_ID) {
        send_event(drained ? UP_PLAY_DONE : UP_PLAY_ABORT, completed, DIR_UNKNOWN);
    }
    int gain = atomic_load(&volume);
    for (int i = 0; i < frame; ++i) {
        if (tone_active && tone + i < 8000) {
            unsigned phase = (tone + i) % 32;
            int triangle = phase < 16 ? (int)phase : 32 - (int)phase;
            outgoing[i] = (int16_t)((triangle - 8) * 560);
        }
        int32_t sample = outgoing[i] * gain / 100;
        i2s_tx[i * 2] = i2s_tx[i * 2 + 1] = sample * 65536;
    }
    return first_pcm;
}

static int16_t mic_sample(int32_t value)
{
    int32_t sample = value >> 14;
    if (sample > INT16_MAX) return INT16_MAX;
    if (sample < INT16_MIN) return INT16_MIN;
    return (int16_t)sample;
}

static bool signal_is_speech(const int16_t *pcm)
{
    if (!vad) return true;
    static int16_t chunk[160];
    static size_t filled;
    static bool speech;
    unsigned speech_chunks = 0, chunks = 0;
    for (int offset = 0; offset < frame;) {
        size_t count = sizeof(chunk) / sizeof(chunk[0]) - filled;
        if (count > frame - offset) count = frame - offset;
        memcpy(chunk + filled, pcm + offset, count * sizeof(int16_t));
        filled += count;
        offset += count;
        if (filled == sizeof(chunk) / sizeof(chunk[0])) {
            ++chunks;
            speech_chunks += vad_process(vad, chunk, SAMPLE_RATE, 10) == VAD_SPEECH;
            filled = 0;
        }
    }
    if (chunks) speech = speech_chunks * 2 >= chunks;
    return speech;
}

static void setup_wake(void)
{
    srmodel_list_t *models = esp_srmodel_init("model");
    const wake_option_t *choice = audio_selected_wake();
    char *name = models ? esp_srmodel_filter(models, ESP_WN_PREFIX, choice->model) : NULL;
    wake = name ? esp_wn_handle_from_name(name) : NULL;
    // The bundle stores many models in flash, but only the selected one is
    // instantiated in PSRAM for inference.
    if (wake) wake_data = wake->create(name, DET_MODE_95);
    if (wake_data) wake_frame = wake->get_samp_chunksize(wake_data);
    if (!wake_data || wake_frame <= 0 || wake_frame > sizeof(wake_pcm) / sizeof(wake_pcm[0]) ||
        wake->get_channel_num(wake_data) != 1 || wake->get_samp_rate(wake_data) != SAMPLE_RATE) {
        if (wake_data) wake->destroy(wake_data);
        wake_data = NULL;
        ESP_LOGE("audio", "WakeNet unavailable. Check that srmodels.bin was flashed.");
    } else {
        ESP_LOGI("audio", "WakeNet ready: %s (%s)", choice->phrase, name);
    }
}

static bool keyword_detected(const int16_t *pcm)
{
    if (!wake_data) return false;
    bool detected = false;
    for (int offset = 0; offset < frame;) {
        int count = wake_frame - wake_used;
        if (count > frame - offset) count = frame - offset;
        memcpy(wake_pcm + wake_used, pcm + offset, count * sizeof(int16_t));
        wake_used += count;
        offset += count;
        if (wake_used == wake_frame) {
            detected |= wake->detect(wake_data, wake_pcm) > 0;
            wake_used = 0;
        }
    }
    return detected;
}

static void audio_io_fault(bool output, uint32_t capture)
{
    taskENTER_CRITICAL(&state_lock);
    if (output) ++telemetry.i2s_tx_errors;
    else ++telemetry.i2s_rx_errors;
    bool cancel = capture || play.active;
    taskEXIT_CRITICAL(&state_lock);
    if (capture) send_event(UP_ABORT, capture, DIR_UNKNOWN);
    if (cancel) interrupt_audio(PLAYBACK_I2S_ERROR);
}

static void audio_task(void *unused)
{
    (void)unused;
    unsigned ring = 0, ring_size = 0, seen_reset = 0;
    int quiet_ms = 0, capture_ms = 0, speech_ms = 0;
    int pending_voiced_ms = 0;
    int block_ms = frame * 1000 / SAMPLE_RATE;
    int64_t echo_until = 0, isolated_until = 0, next_wake = 0, direction_at = 0;
    uint32_t barge_id = 0;
    barge_in_t barge = {0};
    uint32_t last_speech_ms = 0;
    direction_t last_direction = DIR_UNKNOWN;
    bool was_speaking = false;
    bool followup = false;
    direction_t wake_direction = DIR_UNKNOWN;
    unsigned direction_votes[DIR_RIGHT + 1] = {0};
    unsigned barge_votes[DIR_RIGHT + 1] = {0};
    uint32_t active = 0;
    while (true) {
        size_t bytes = 0;
        if (i2s_channel_read(rx, i2s_rx, frame * 8, &bytes, 100) != ESP_OK || bytes != frame * 8) {
            audio_io_fault(false, active);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        int64_t now = esp_timer_get_time();
        bool playing, speaking, isolated;
        uint32_t playback_id;
        uint32_t first_pcm = fill_output(now, barge.silence_ms ? barge_id : 0, &playback_id,
                                         &playing, &speaking, &isolated);
        if (i2s_channel_write(tx, i2s_tx, frame * 8, &bytes, 40) != ESP_OK || bytes != frame * 8) {
            audio_io_fault(true, active);
            continue;
        }
        if (first_pcm) {
            uplink_t start = {.type = UP_PLAY_BEGIN, .id = first_pcm, .at_ms = esp_timer_get_time() / 1000};
            cloud_uplink(&start);
        }

        taskENTER_CRITICAL(&reference_lock);
        unsigned back = (unsigned)robot_config.ref_delay;
        memcpy(reference, references[(reference_index + REF_HISTORY - back) % REF_HISTORY], frame * 2);
        taskEXIT_CRITICAL(&reference_lock);
        for (int i = 0; i < frame; ++i) {
            mic_l[i] = mic_sample(i2s_rx[i * 2]);
            mic_r[i] = mic_sample(i2s_rx[i * 2 + 1]);
            mono[i] = (int16_t)(((int)mic_l[i] + mic_r[i]) / 2);
        }
        // Playback reference and microphone samples share one I2S clock.
        if (aec) aec_process(aec, mono, reference, clean);
        if (was_speaking && !speaking) echo_until = now + ECHO_TAIL_US;
        was_speaking = speaking;
        if (isolated) isolated_until = now + ECHO_TAIL_US;
        bool allow_voice_trigger = now >= isolated_until;
        bool echo_path = speaking || now < echo_until;
        bool ready = network_ready();
        bool can_barge = ready && aec && speaking && !isolated && allow_voice_trigger && robot_config.barge;
        if (!can_barge || playback_id != barge_id) {
            barge = (barge_in_t){0};
            memset(barge_votes, 0, sizeof(barge_votes));
            barge_id = can_barge ? playback_id : 0;
        }
        int reference_rms = can_barge ? pcm_rms(reference, frame) : 0;
        barge_in_reference(&barge, reference_rms, block_ms);
        bool near_check = barge_in_near_mic(&barge);
        const int16_t *voice = echo_path && aec && !near_check ? clean : mono;
        int level = pcm_rms(voice, frame);
        bool speech_like = signal_is_speech(voice);
        bool voiced = speech_like && level >= robot_config.vad_rms;
        int left = pcm_rms(mic_l, frame), right = pcm_rms(mic_r, frame);
        listen_mode_t forced;
        uint32_t old = 0;
        bool reset = false;
        taskENTER_CRITICAL(&state_lock);
        if (seen_reset != reset_capture) {
            seen_reset = reset_capture;
            reset = true;
            old = interrupted_id;
            interrupted_id = 0;
        }
        forced = force_listen;
        force_listen = LISTEN_NONE;
        telemetry.left_rms = left;
        telemetry.right_rms = right;
        telemetry.speech_rms = level;
        taskEXIT_CRITICAL(&state_lock);
        if (reset) {
            if (active) send_event(UP_ABORT, active, DIR_UNKNOWN);
            if (old) send_event(UP_INTERRUPT, old, DIR_UNKNOWN);
            active = 0;
            followup = false;
            pending_voiced_ms = 0;
            barge = (barge_in_t){0};
            memset(barge_votes, 0, sizeof(barge_votes));
            ring_size = 0;
            wake_used = 0;
            next_wake = now + WAKE_DRAIN_US;
        }

        if (!active && !playing && echo_path) {
            ring_size = 0;
        } else {
            memcpy(preroll + ring * frame, voice, frame * sizeof(int16_t));
            ring = (ring + 1) % preroll_capacity;
            if (ring_size < preroll_capacity) ++ring_size;
        }

        direction_t frame_direction = DIR_UNKNOWN;
        if (!echo_path && speech_like && left > 12 && right > 12 && level >= robot_config.vad_rms) {
            frame_direction = estimate_direction(mic_l, mic_r, frame, robot_config.mic_spacing_mm);
            if (frame_direction != DIR_UNKNOWN) {
                last_direction = frame_direction;
                direction_at = now;
            }
        }
        direction_t live_direction = direction_at && now - direction_at < 950000 ?
                                     last_direction : DIR_UNKNOWN;
        taskENTER_CRITICAL(&state_lock);
        telemetry.direction = !echo_path ? live_direction : DIR_UNKNOWN;
        taskEXIT_CRITICAL(&state_lock);

        if (active) {
            ++direction_votes[frame_direction];
            if (now < next_wake) keyword_detected(wake_silence);
            capture_ms += block_ms;
            if (voiced) {
                last_speech_ms = now / 1000;
                speech_ms += block_ms;
                quiet_ms = 0;
            } else {
                quiet_ms += block_ms;
            }
            bool uploaded = ready && send_pcm(active, voice);
            int silence = speech_ms < MIN_CAPTURE_VOICE_MS ? 2200 : 420;
            if (!uploaded || quiet_ms >= silence || capture_ms >= 11900) {
                taskENTER_CRITICAL(&state_lock);
                telemetry.listening = false;
                bool same = turn_id == active;
                bool valid = same && uploaded && speech_ms >= MIN_CAPTURE_VOICE_MS;
                if (valid && dialog_active) pending_voice_until = now + PENDING_VOICE_US;
                taskEXIT_CRITICAL(&state_lock);
                uplink_t end = {.type = UP_END, .id = active, .at_ms = last_speech_ms,
                                .direction = speech_direction(direction_votes)};
                if (!valid || !cloud_uplink(&end)) {
                    send_event(UP_ABORT, active, DIR_UNKNOWN);
                    audio_cancel_turn(active);
                }
                active = 0;
                ring_size = 0;
                next_wake = now + 350000;
            }
            continue;
        }

        // Drain the model's previous audio through its input. The ESP-SR
        // clean callback can dereference a null queue in the WakeNet9 build.
        int threshold = robot_config.barge_rms;
        int weaker_mic = left < right ? left : right;
        // AEC supplies the onset level. Both mics must hear the candidate,
        // but an uncertain angle should never hide a spoken interruption.
        int min_level = near_check ? barge_in_confirm_level(threshold) : threshold;
        bool barge_speech = can_barge && speech_like && weaker_mic >= min_level / 2;
        if (near_check && barge_speech && level >= min_level) {
            direction_t confirmed_direction = estimate_direction(mic_l, mic_r, frame, robot_config.mic_spacing_mm);
            ++barge_votes[confirmed_direction];
        }
        bool was_checking = barge.silence_ms != 0;
        bool voice_barge = can_barge && barge_in_step(&barge, level, barge_speech,
                                                      threshold, block_ms);
        if (was_checking && !barge.silence_ms) memset(barge_votes, 0, sizeof(barge_votes));
        // A wake word picked up from the rover's own reply must not bypass
        // human-voice confirmation while the speaker has queued audio.
        bool check_keyword = allow_voice_trigger && now >= next_wake &&
                             !playing && !echo_path;
        bool keyword = keyword_detected(check_keyword ? voice : wake_silence) && check_keyword;
        bool after_cue = followup && !speaking && allow_voice_trigger;
        taskENTER_CRITICAL(&state_lock);
        if (!was_checking && barge.silence_ms && play.active && play.id == playback_id) ++telemetry.barge_checks;
        if (seen_reset != reset_capture) {
            taskEXIT_CRITICAL(&state_lock);
            continue;
        }
        voice_barge = voice_barge && play.active && play.id == playback_id;
        // Accept a new utterance while waiting for ASR/Flash or the first
        // synthesized samples, once the previous capture has gone quiet.
        bool pending_question = ready && dialog_active && now < pending_voice_until &&
            now >= next_wake && !echo_path && allow_voice_trigger &&
            (!play.active || (!play.started && !play.isolated)) && !forced && !keyword;
        pending_voiced_ms = pending_question && voiced ? pending_voiced_ms + block_ms : 0;
        bool replace_pending = pending_voiced_ms >= DIALOG_TRIGGER_MS;
        bool next_question = dialog_wait_step(&dialog_window, now,
            ready && !playing && !echo_path && allow_voice_trigger && !forced && !keyword, voiced, block_ms);
        if (forced || keyword || voice_barge || after_cue || next_question || replace_pending) {
            direction_t direction = after_cue ? wake_direction : voice_barge ? speech_direction(barge_votes) :
                                    !echo_path ? live_direction : DIR_UNKNOWN;
            bool acknowledge = keyword && !forced && !echo_path && !after_cue && audio_volume() > 0;
            followup = false;
            telemetry.direction = direction;
            uint32_t previous = turn_id;
            clear_play(forced ? PLAYBACK_MANUAL : voice_barge && !keyword ? PLAYBACK_VOICE : PLAYBACK_WAKE_WORD);
            turn_id = 0;
            dialog_window = (dialog_window_t){0};
            pending_voice_until = 0;
            if (forced || keyword || voice_barge) dialog_active = !forced && ready;
            if (ready && !acknowledge) {
                active = turn_id = next_turn_locked();
                telemetry.listening = true;
            }
            taskEXIT_CRITICAL(&state_lock);
            if (voice_barge) {
                was_speaking = false;
                echo_until = 0;
            }
            nav_stop(false);
            if (!next_question && (previous || echo_path)) send_event(UP_INTERRUPT, previous, DIR_UNKNOWN);
            if (acknowledge) {
                audio_wake_feedback();
                followup = ready;
                wake_direction = direction;
            } else if (ready) {
                if (voice_barge) memcpy(direction_votes, barge_votes, sizeof(direction_votes));
                else {
                    memset(direction_votes, 0, sizeof(direction_votes));
                    ++direction_votes[frame_direction];
                }
                uplink_t wake_packet = {.type = UP_WAKE, .id = active, .direction = direction,
                                        .asr_only = forced == LISTEN_ASR_ONLY};
                bool ok = cloud_uplink(&wake_packet);
                unsigned retained = 0;
                if (!forced && !after_cue) {
                    unsigned limit = PREROLL;
                    if (voice_barge) {
                        // Include speech that led up to the probe without sending
                        // unrelated earlier audio from the speaker to recognition.
                        limit = (barge.silence_ms + barge.candidate_ms + barge.candidate_gaps_ms +
                                 BARGE_PREROLL_LEAD_MS +
                                 block_ms - 1) / block_ms;
                    }
                    retained = ring_size < limit ? ring_size : limit;
                    for (unsigned i = 0; ok && i < retained; ++i) {
                        unsigned index = (ring + preroll_capacity - retained + i) % preroll_capacity;
                        ok = send_pcm(active, preroll + index * frame);
                    }
                }
                if (!ok) {
                    send_event(UP_ABORT, active, DIR_UNKNOWN);
                    audio_cancel_turn(active);
                    active = 0;
                }
                capture_ms = (int)retained * block_ms;
                quiet_ms = 0;
                speech_ms = voice_barge ? BARGE_VOICE_START_MS + BARGE_VOICE_CONFIRM_MS :
                            next_question || replace_pending ? DIALOG_TRIGGER_MS : 0;
                last_speech_ms = now / 1000;
            }
            wake_used = 0;
            pending_voiced_ms = 0;
            barge = (barge_in_t){0};
            memset(barge_votes, 0, sizeof(barge_votes));
            ring_size = 0;
            next_wake = now + WAKE_DRAIN_US;
        } else {
            taskEXIT_CRITICAL(&state_lock);
        }
    }
}

bool audio_start(void)
{
    audio_set_volume(robot_config.volume);
    aec_config_t echo = {
        .mic_num = 1, .ref_num = 1, .out_num = 1, .filter_length = 4, .sample_rate = SAMPLE_RATE,
        .caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, .mode = AEC_MODE_FD_LOW_COST,
        .nlp_level = AEC_NLP_LEVEL_NORMAL,
    };
    aec = aec_create_from_config(&echo);
    vad = vad_create(VAD_MODE_1);
    setup_wake();
    frame = aec ? aec_get_chunksize(aec) : 256;
    if (frame <= 0 || frame > MAX_FRAME) return false;
    preroll_capacity = (SAMPLE_RATE * BARGE_PREROLL_MS / 1000 + frame - 1) / frame;
    preroll = heap_caps_malloc(preroll_capacity * frame * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!preroll) return false;
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    // Stereo 32-bit samples take 8 bytes. A 512-sample AEC block does
    // not fit a 4092-byte descriptor, so use two equal 256-sample buffers.
    bool split_dma_block = frame * 8 > 4092;
    chan.dma_desc_num = split_dma_block ? 6 : 3;
    chan.dma_frame_num = split_dma_block ? frame / 2 : frame;
    chan.auto_clear_after_cb = true;
    if (i2s_new_channel(&chan, &tx, &rx) != ESP_OK) return false;
    i2s_std_config_t config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {.mclk = I2S_GPIO_UNUSED, .bclk = PIN_BCLK, .ws = PIN_WS,
                     .dout = PIN_SPEAKER, .din = PIN_MIC},
    };
    config.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
    i2s_event_callbacks_t callbacks = {.on_sent = tx_complete};
    if (i2s_channel_init_std_mode(tx, &config) != ESP_OK ||
        i2s_channel_init_std_mode(rx, &config) != ESP_OK ||
        i2s_channel_register_event_callback(tx, &callbacks, NULL) != ESP_OK ||
        i2s_channel_enable(tx) != ESP_OK || i2s_channel_enable(rx) != ESP_OK) return false;

    taskENTER_CRITICAL(&state_lock);
    telemetry.aec = aec != NULL;
    telemetry.wake_ready = wake_data != NULL;
    initialized = true;
    taskEXIT_CRITICAL(&state_lock);
    if (xTaskCreatePinnedToCore(audio_task, "audio", 7168, NULL, 6, NULL, 1) != pdPASS) {
        taskENTER_CRITICAL(&state_lock);
        initialized = false;
        taskEXIT_CRITICAL(&state_lock);
        return false;
    }
    ESP_LOGI("audio", "Dual INMP441, %d-sample frames, %lu-sample DMA, AEC %s",
             frame, (unsigned long)chan.dma_frame_num, aec ? "ready" : "unavailable");
    return true;
}
