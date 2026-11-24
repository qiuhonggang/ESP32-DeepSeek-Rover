#include "robot.h"
#include "nav.h"
#include "esp_wn_models.h"
#include "model_path.h"
#include <math.h>
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
#define PREROLL 9
#define REF_HISTORY 4
#define TEST_TONE_ID UINT32_MAX
#define PLAY_DRAIN_US 85000

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
static bool force_listen;
static uint32_t interrupted_id;
static int frame;
static bool initialized;
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE reference_lock = portMUX_INITIALIZER_UNLOCKED;
static audio_status_t telemetry;
static uint32_t turn_id;
static uint32_t next_id;
static unsigned reset_capture;
static struct {
    int16_t pcm[PLAY_CAPACITY];
    size_t read, length;
    uint32_t id;
    int64_t start, last_rx, drain_after;
    unsigned tone_sample;
    bool active, ended, started;
} play;
static int16_t references[REF_HISTORY][MAX_FRAME];
static unsigned reference_index;
static int16_t preroll[PREROLL][MAX_FRAME];
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
    taskEXIT_CRITICAL(&state_lock);
    return copy;
}

static bool send_event(uplink_type_t type, uint32_t id, direction_t direction);

static void clear_play(void)
{
    play.active = false;
    play.length = 0;
    play.ended = false;
    play.started = false;
}

void audio_interrupt(void)
{
    taskENTER_CRITICAL(&state_lock);
    interrupted_id = turn_id;
    force_listen = false;
    clear_play();
    ++reset_capture;
    turn_id = 0;
    telemetry.listening = false;
    taskEXIT_CRITICAL(&state_lock);
}

bool audio_listen_once(void)
{
    if (!network_ready()) return false;
    taskENTER_CRITICAL(&state_lock);
    bool accepted = initialized && !telemetry.listening && !force_listen;
    if (accepted) force_listen = true;
    taskEXIT_CRITICAL(&state_lock);
    return accepted;
}

void audio_cancel_turn(uint32_t id)
{
    taskENTER_CRITICAL(&state_lock);
    if (turn_id == id && id != 0) {
        turn_id = 0;
        ++reset_capture;
        telemetry.listening = false;
    }
    if (play.active && play.id == id) clear_play();
    taskEXIT_CRITICAL(&state_lock);
}

bool audio_turn_is_current(uint32_t id)
{
    taskENTER_CRITICAL(&state_lock);
    bool current = id && id == turn_id && !telemetry.listening;
    taskEXIT_CRITICAL(&state_lock);
    return current;
}

bool audio_begin_play(uint32_t id)
{
    taskENTER_CRITICAL(&state_lock);
    bool accepted = initialized && !telemetry.listening && !play.active &&
                    (id == 0 || id == TEST_TONE_ID || id == turn_id);
    if (accepted) {
        play.id = id;
        play.read = play.length = 0;
        play.tone_sample = 0;
        play.last_rx = esp_timer_get_time();
        play.start = 0;
        play.drain_after = 0;
        play.active = true;
        play.started = false;
        play.ended = id == TEST_TONE_ID;
    }
    taskEXIT_CRITICAL(&state_lock);
    return accepted;
}

bool audio_test_tone(void)
{
    return audio_begin_play(TEST_TONE_ID);
}

bool audio_push_play(const void *data, size_t size)
{
    if (!size || size % 2 || size > MAX_WIRE_BYTES) return false;
    taskENTER_CRITICAL(&state_lock);
    size_t count = size / sizeof(int16_t);
    bool accepted = play.active && !play.ended && count <= PLAY_CAPACITY - play.length;
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
    if (!event->dma_buf || event->size != frame * 8) return false;
    const int32_t *samples = event->dma_buf;
    portENTER_CRITICAL_ISR(&reference_lock);
    unsigned index = ++reference_index % REF_HISTORY;
    for (int i = 0; i < frame; ++i) references[index][i] = (int16_t)(samples[i * 2] >> 16);
    portEXIT_CRITICAL_ISR(&reference_lock);
    return false;
}

static void fill_output(int64_t now, bool *speaking, int64_t *play_start)
{
    uint32_t completed = TEST_TONE_ID;
    bool drained = false;
    unsigned tone = 0;
    bool tone_active = false;
    memset(outgoing, 0, frame * sizeof(int16_t));
    taskENTER_CRITICAL(&state_lock);
    if (play.active) {
        if (play.id == TEST_TONE_ID && play.tone_sample < 8000) {
            tone_active = true;
            tone = play.tone_sample;
            play.tone_sample += frame;
            play.drain_after = now + PLAY_DRAIN_US;
            if (!play.started) play.start = now;
            play.started = true;
        } else if (play.length && (play.started || play.ended || play.length >= (size_t)(frame * 4))) {
            size_t count = play.length < frame ? play.length : frame;
            size_t first = PLAY_CAPACITY - play.read;
            if (first > count) first = count;
            memcpy(outgoing, play.pcm + play.read, first * 2);
            if (count > first) memcpy(outgoing + first, play.pcm, (count - first) * 2);
            play.length -= count;
            play.read = (play.read + count) % PLAY_CAPACITY;
            play.drain_after = now + PLAY_DRAIN_US;
            if (!play.started) play.start = now;
            play.started = true;
        } else if ((play.ended && !play.length && now >= play.drain_after) ||
                   now - play.last_rx > 1600000) {
            completed = play.id;
            drained = play.ended && !play.length && now >= play.drain_after;
            clear_play();
        }
    }
    *speaking = play.active && play.started;
    *play_start = play.start;
    telemetry.speaking = *speaking;
    taskEXIT_CRITICAL(&state_lock);

    if (completed != TEST_TONE_ID) {
        send_event(drained ? UP_PLAY_DONE : UP_PLAY_ABORT, completed, DIR_UNKNOWN);
    }
    for (int i = 0; i < frame; ++i) {
        if (tone_active && tone + i < 8000) {
            unsigned phase = (tone + i) % 32;
            int triangle = phase < 16 ? (int)phase : 32 - (int)phase;
            outgoing[i] = (int16_t)((triangle - 8) * 560);
        }
        int32_t sample = outgoing[i] * robot_config.volume / 100;
        i2s_tx[i * 2] = i2s_tx[i * 2 + 1] = sample * 65536;
    }
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
    for (int offset = 0; offset < frame;) {
        size_t count = sizeof(chunk) / sizeof(chunk[0]) - filled;
        if (count > frame - offset) count = frame - offset;
        memcpy(chunk + filled, pcm + offset, count * sizeof(int16_t));
        filled += count;
        offset += count;
        if (filled == sizeof(chunk) / sizeof(chunk[0])) {
            speech = vad_process(vad, chunk, SAMPLE_RATE, 10) == VAD_SPEECH;
            filled = 0;
        }
    }
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

static void audio_task(void *unused)
{
    (void)unused;
    unsigned ring = 0, ring_size = 0, seen_reset = 0;
    int loud_ms = 0, quiet_ms = 0, capture_ms = 0, speech_ms = 0;
    int block_ms = frame * 1000 / SAMPLE_RATE;
    int64_t echo_until = 0, next_wake = 0, direction_at = 0;
    direction_t last_direction = DIR_UNKNOWN;
    bool was_speaking = false;
    uint32_t active = 0;
    while (true) {
        size_t bytes = 0;
        if (i2s_channel_read(rx, i2s_rx, frame * 8, &bytes, 100) != ESP_OK || bytes != frame * 8) {
            if (active) send_event(UP_ABORT, active, DIR_UNKNOWN);
            audio_interrupt();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        int64_t now = esp_timer_get_time(), play_start;
        bool speaking;
        fill_output(now, &speaking, &play_start);
        if (i2s_channel_write(tx, i2s_tx, frame * 8, &bytes, 40) != ESP_OK || bytes != frame * 8) {
            if (active) send_event(UP_ABORT, active, DIR_UNKNOWN);
            audio_interrupt();
            continue;
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
        if (was_speaking && !speaking) echo_until = now + 90000;
        was_speaking = speaking;
        bool echo_path = speaking || now < echo_until;
        const int16_t *voice = echo_path && aec ? clean : mono;
        int level = pcm_rms(voice, frame);
        bool speech_like = signal_is_speech(voice);
        int left = pcm_rms(mic_l, frame), right = pcm_rms(mic_r, frame);
        bool ready = network_ready();
        bool forced;
        uint32_t old = 0;
        bool reset = false;
        taskENTER_CRITICAL(&state_lock);
        if (seen_reset != reset_capture) {
            seen_reset = reset_capture;
            reset = true;
            old = interrupted_id;
        }
        forced = force_listen;
        force_listen = false;
        telemetry.left_rms = left;
        telemetry.right_rms = right;
        telemetry.speech_rms = level;
        taskEXIT_CRITICAL(&state_lock);
        if (reset) {
            if (active) send_event(UP_ABORT, active, DIR_UNKNOWN);
            if (old) send_event(UP_INTERRUPT, old, DIR_UNKNOWN);
            active = 0;
            loud_ms = 0;
            ring_size = 0;
            if (wake_data) wake->clean(wake_data);
            wake_used = 0;
        }

        memcpy(preroll[ring], voice, frame * 2);
        ring = (ring + 1) % PREROLL;
        if (ring_size < PREROLL) ++ring_size;

        if (active) {
            capture_ms += block_ms;
            if (speech_like && level >= robot_config.vad_rms) {
                speech_ms += block_ms;
                quiet_ms = 0;
            } else {
                quiet_ms += block_ms;
            }
            bool uploaded = ready && send_pcm(active, voice);
            int silence = speech_ms < 130 ? 2200 : 670;
            if (!uploaded || quiet_ms >= silence || capture_ms >= 11900) {
                taskENTER_CRITICAL(&state_lock);
                telemetry.listening = false;
                bool same = turn_id == active;
                taskEXIT_CRITICAL(&state_lock);
                if (!same || !uploaded || speech_ms < 130 || !send_event(UP_END, active, DIR_UNKNOWN)) {
                    send_event(UP_ABORT, active, DIR_UNKNOWN);
                    audio_cancel_turn(active);
                }
                active = 0;
                loud_ms = 0;
                ring_size = 0;
                next_wake = now + 350000;
            }
            continue;
        }

        if (!echo_path && speech_like && left > 12 && right > 12 && level >= robot_config.vad_rms) {
            direction_t direction = estimate_direction(mic_l, mic_r, frame, robot_config.mic_spacing_mm);
            if (direction != DIR_UNKNOWN) {
                last_direction = direction;
                direction_at = now;
            }
        }
        bool keyword = now >= next_wake && (!echo_path || aec) && keyword_detected(voice);
        int threshold = robot_config.barge_rms + pcm_rms(reference, frame) / 12;
        bool barge = robot_config.barge && echo_path && aec && speech_like &&
                     level >= threshold && (!speaking || now - play_start >= 100000);
        loud_ms = barge ? loud_ms + block_ms : 0;
        if (forced || keyword || loud_ms >= 96) {
            direction_t direction = !echo_path && direction_at && now - direction_at < 700000 ?
                                    last_direction : DIR_UNKNOWN;
            uint32_t previous;
            taskENTER_CRITICAL(&state_lock);
            telemetry.direction = direction;
            previous = turn_id;
            clear_play();
            turn_id = 0;
            if (ready) {
                if (++next_id > INT32_MAX) next_id = 1;
                active = turn_id = next_id;
                telemetry.listening = true;
            }
            taskEXIT_CRITICAL(&state_lock);
            nav_stop(false);
            if (previous || echo_path) send_event(UP_INTERRUPT, previous, DIR_UNKNOWN);
            if (ready) {
                bool ok = send_event(UP_WAKE, active, direction);
                if (!forced) {
                    for (unsigned i = 0; ok && i < ring_size; ++i) {
                        unsigned index = (ring + PREROLL - ring_size + i) % PREROLL;
                        ok = send_pcm(active, preroll[index]);
                    }
                }
                if (!ok) {
                    send_event(UP_ABORT, active, DIR_UNKNOWN);
                    audio_cancel_turn(active);
                    active = 0;
                }
                capture_ms = forced ? 0 : (int)ring_size * block_ms;
                quiet_ms = 0;
                speech_ms = 0;
            }
            if (wake_data) wake->clean(wake_data);
            wake_used = 0;
            loud_ms = 0;
            ring_size = 0;
            next_wake = now + 950000;
        }
    }
}

bool audio_start(void)
{
    aec_config_t echo = {
        .mic_num = 1, .ref_num = 1, .out_num = 1, .filter_length = 4, .sample_rate = SAMPLE_RATE,
        .caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, .mode = AEC_MODE_FD_LOW_COST,
        .nlp_level = AEC_NLP_LEVEL_AGGR,
    };
    aec = aec_create_from_config(&echo);
    vad = vad_create(VAD_MODE_1);
    setup_wake();
    frame = aec ? aec_get_chunksize(aec) : 256;
    if (frame <= 0 || frame > MAX_FRAME) return false;
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan.dma_desc_num = 3;
    chan.dma_frame_num = frame;
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
    ESP_LOGI("audio", "Dual INMP441, %d-sample frames, AEC %s", frame, aec ? "ready" : "unavailable");
    return true;
}
