#include "cloud.h"
#include "audio_wire.h"
#include "harness.h"
#include "intent.h"
#include "nav.h"
#include "robot.h"
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define ASR_URL "https://api.deepseek.com/v1/tools/asr"
#define FLASH_URL "https://api.deepseek.com/chat/completions"
#define TTS_URL "https://api.deepseek.com/v1/tools/tts"
#define TTS_WS_URL "wss://api.deepseek.com/v1/tools/tts"
#define MAX_RECORD_BYTES (SAMPLE_RATE * 2 * 12)
#define JSON_CAPACITY 8192
#define TTS_MAX_BYTES (24000 * 2 * 25)
#define BOUNDARY "ESP32Rover-a77b2f2d16b64050"
static const char *TAG = "cloud";

typedef enum { JOB_VOICE, JOB_ASR_ONLY, JOB_TEXT_CHAT, JOB_TEXT_TTS, JOB_TEXT_STREAM } job_type_t;
typedef struct {
    uint32_t id;
    size_t bytes;
    int64_t captured_at;
    uint32_t speech_end_ms;
    direction_t direction;
    job_type_t type;
    uint8_t data[];
} utterance_t;

typedef struct { const void *data; size_t length; } part_t;
typedef struct { esp_http_client_handle_t client; int http_status; } post_t;

static utterance_t *recording;
static QueueHandle_t jobs;
static SemaphoreHandle_t status_lock;
static char phase[32] = "idle", last_text[243], last_reply[301];
static int last_http;
static hardware_context_t last_hardware;
static bool has_hardware;
static atomic_int voice_pace = PACE_NORMAL;
static atomic_uint playback_done, playback_failed;
static atomic_bool initialized, text_debug_busy;
static atomic_int flash_first_ms, flash_complete_ms, tts_first_ms;
static atomic_int asr_ms, first_play_ms, tts_ws_ms;
static atomic_int prompt_tokens = -1, completion_tokens = -1, cached_tokens = -1;
static atomic_uint timing_id, timing_origin_ms;
static atomic_bool http_reused;
// Owned by the cloud task and kept only between ASR and the following Flash request.
static esp_http_client_handle_t asr_connection;

static int elapsed_ms(uint32_t start)
{
    return (int)((uint32_t)(esp_timer_get_time() / 1000) - start);
}

static void record_usage(const cJSON *body)
{
    const cJSON *usage = cJSON_GetObjectItemCaseSensitive(body, "usage");
    if (!cJSON_IsObject(usage)) return;
    const cJSON *input = cJSON_GetObjectItemCaseSensitive(usage, "prompt_tokens");
    const cJSON *output = cJSON_GetObjectItemCaseSensitive(usage, "completion_tokens");
    const cJSON *cached = cJSON_GetObjectItemCaseSensitive(usage, "prompt_cache_hit_tokens");
    if (cJSON_IsNumber(input)) atomic_store(&prompt_tokens, input->valueint);
    if (cJSON_IsNumber(output)) atomic_store(&completion_tokens, output->valueint);
    if (cJSON_IsNumber(cached)) atomic_store(&cached_tokens, cached->valueint);
}

static void status(const char *state, int http, const char *text, const char *answer)
{
    xSemaphoreTake(status_lock, portMAX_DELAY);
    strlcpy(phase, state, sizeof(phase));
    last_http = http;
    if (text) text_copy(last_text, sizeof(last_text), text);
    if (answer) text_copy(last_reply, sizeof(last_reply), answer);
    xSemaphoreGive(status_lock);
}

static hardware_context_t snapshot_hardware(direction_t direction)
{
    hardware_context_t hardware = {
        .speech_direction = direction, .chassis = motion_status(), .volume = audio_volume(),
        .max_pwm = robot_config.speed, .pace = atomic_load(&voice_pace),
    };
    xSemaphoreTake(status_lock, portMAX_DELAY);
    last_hardware = hardware;
    has_hardware = true;
    xSemaphoreGive(status_lock);
    return hardware;
}

cJSON *cloud_json(void)
{
    cJSON *json = cJSON_CreateObject();
    if (!json) return NULL;
    xSemaphoreTake(status_lock, portMAX_DELAY);
    cJSON_AddStringToObject(json, "phase", phase);
    cJSON_AddNumberToObject(json, "http", last_http);
    cJSON_AddStringToObject(json, "transcript", last_text);
    cJSON_AddStringToObject(json, "reply", last_reply);
    cJSON_AddBoolToObject(json, "ready", network_ready() && atomic_load(&initialized));
    cJSON_AddBoolToObject(json, "debug_busy", atomic_load(&text_debug_busy));
    cJSON_AddNumberToObject(json, "flash_first_ms", atomic_load(&flash_first_ms));
    cJSON_AddNumberToObject(json, "flash_complete_ms", atomic_load(&flash_complete_ms));
    cJSON_AddNumberToObject(json, "tts_first_ms", atomic_load(&tts_first_ms));
    cJSON_AddNumberToObject(json, "asr_ms", atomic_load(&asr_ms));
    cJSON_AddNumberToObject(json, "first_play_ms", atomic_load(&first_play_ms));
    cJSON_AddNumberToObject(json, "tts_ws_ms", atomic_load(&tts_ws_ms));
    cJSON_AddBoolToObject(json, "http_reused", atomic_load(&http_reused));
    cJSON_AddNumberToObject(json, "prompt_tokens", atomic_load(&prompt_tokens));
    cJSON_AddNumberToObject(json, "completion_tokens", atomic_load(&completion_tokens));
    cJSON_AddNumberToObject(json, "cached_tokens", atomic_load(&cached_tokens));
    cJSON_AddStringToObject(json, "voice_pace", voice_pace_name(atomic_load(&voice_pace)));
    if (has_hardware) cJSON_AddItemToObject(json, "hardware", harness_hardware_json(&last_hardware));
    xSemaphoreGive(status_lock);
    return json;
}

static void release_job(utterance_t *job)
{
    if (job->type >= JOB_TEXT_CHAT) {
        atomic_store(&text_debug_busy, false);
    }
    free(job);
}

bool cloud_debug_text(const char *text, const char *command)
{
    if (!text || !text[0] || !atomic_load(&initialized) || !network_ready()) return false;
    bool speak_only = !strcmp(command, "debug_tts");
    size_t bytes = strlen(text);
    if (bytes > (speak_only ? sizeof(last_reply) - 1 : sizeof(last_text) - 1) ||
        atomic_exchange(&text_debug_busy, true)) return false;
    utterance_t *job = malloc(sizeof(*job) + bytes + 1);
    if (!job) {
        atomic_store(&text_debug_busy, false);
        return false;
    }
    *job = (utterance_t){.type = speak_only ? JOB_TEXT_TTS :
        !strcmp(command, "debug_stream") ? JOB_TEXT_STREAM : JOB_TEXT_CHAT};
    memcpy(job->data, text, bytes + 1);
    if (uxQueueSpacesAvailable(jobs)) {
        job->id = audio_begin_debug_turn();
        if (job->id && xQueueSend(jobs, &job, 0) == pdTRUE) return true;
        if (job->id) audio_cancel_turn(job->id);
    }
    release_job(job);
    return false;
}

bool cloud_uplink(const uplink_t *packet)
{
    if (!atomic_load(&initialized)) return false;
    if (packet->type == UP_PLAY_BEGIN) {
        if (packet->id == atomic_load(&timing_id)) {
            atomic_store(&first_play_ms, (int)(packet->at_ms - atomic_load(&timing_origin_ms)));
        }
        return true;
    }
    if (packet->type == UP_PLAY_DONE || packet->type == UP_PLAY_ABORT) {
        if (packet->type == UP_PLAY_DONE) atomic_store(&playback_done, packet->id);
        else atomic_store(&playback_failed, packet->id);
        return true;
    }
    // Recording ownership is confined to the I2S/audio task. The job transfers
    // ownership to the HTTPS task only after its audio_end.
    if (packet->type == UP_WAKE) {
        free(recording);
        recording = NULL;
        if (!network_ready()) return false;
        recording = heap_caps_malloc(sizeof(*recording) + MAX_RECORD_BYTES, MALLOC_CAP_SPIRAM);
        if (!recording) return false;
        recording->id = packet->id;
        recording->type = packet->asr_only ? JOB_ASR_ONLY : JOB_VOICE;
        recording->bytes = 0;
        xSemaphoreTake(status_lock, portMAX_DELAY);
        has_hardware = false;
        xSemaphoreGive(status_lock);
        status("listening", 0, "", "");
        return true;
    }
    if (packet->type == UP_INTERRUPT || packet->type == UP_ABORT) {
        if (recording && (packet->id == 0 || packet->id == recording->id)) {
            free(recording);
            recording = NULL;
        }
        status(packet->type == UP_INTERRUPT ? "interrupted" : "audio_abort", 0, NULL, NULL);
        return true;
    }
    if (!recording || packet->id != recording->id) return false;
    if (packet->type == UP_AUDIO) {
        if (packet->bytes == 0 || recording->bytes + packet->bytes > MAX_RECORD_BYTES) return false;
        memcpy(recording->data + recording->bytes, packet->pcm, packet->bytes);
        recording->bytes += packet->bytes;
        return true;
    }
    if (packet->type == UP_END) {
        utterance_t *new_job = recording;
        recording = NULL;
        new_job->captured_at = esp_timer_get_time();
        new_job->speech_end_ms = packet->at_ms;
        new_job->direction = packet->direction;
        if (new_job->bytes < 4800 || !network_ready()) {
            free(new_job);
            status("audio_abort", 0, NULL, NULL);
            return false;
        }
        if (xQueueSend(jobs, &new_job, 0) == pdTRUE) return true;
        utterance_t *stale;
        if (xQueueReceive(jobs, &stale, 0) == pdTRUE) release_job(stale);
        if (xQueueSend(jobs, &new_job, 0) == pdTRUE) return true;
        free(new_job);
    }
    return false;
}

static bool current(uint32_t id)
{
    return network_ready() && audio_turn_is_current(id);
}

static bool write_parts(esp_http_client_handle_t client, const part_t *parts, size_t count, uint32_t id)
{
    for (size_t i = 0; i < count; ++i) {
        const char *data = parts[i].data;
        size_t sent = 0;
        while (sent < parts[i].length && current(id)) {
            size_t left = parts[i].length - sent;
            int chunk = left > 4096 ? 4096 : (int)left;
            int written = esp_http_client_write(client, data + sent, chunk);
            if (written <= 0) return false;
            sent += written;
        }
        if (sent != parts[i].length) return false;
    }
    return true;
}

static post_t open_post(const char *url, const char *type, const part_t *parts,
                        size_t count, uint32_t id)
{
    post_t result = {0};
    if (!current(id)) return result;
    size_t length = 0;
    for (size_t i = 0; i < count; ++i) length += parts[i].length;
    esp_http_client_config_t config = {
        .url = url, .method = HTTP_METHOD_POST, .timeout_ms = 7000,
        .transport_type = HTTP_TRANSPORT_OVER_SSL, .crt_bundle_attach = esp_crt_bundle_attach,
        .disable_auto_redirect = true, .buffer_size = 4096, .buffer_size_tx = 4096,
    };
    char authorization[sizeof(robot_config.api_key) + 8];
    snprintf(authorization, sizeof(authorization), "Bearer %s", robot_config.api_key);
    esp_http_client_handle_t client = !strcmp(url, FLASH_URL) ? asr_connection : NULL;
    bool reused = client != NULL;
    if (reused) asr_connection = NULL;
    for (;;) {
        if (!client) client = esp_http_client_init(&config);
        if (!client) break;
        bool sent = (!reused || esp_http_client_set_url(client, url) == ESP_OK) &&
            esp_http_client_set_header(client, "Authorization", authorization) == ESP_OK &&
            esp_http_client_set_header(client, "Content-Type", type) == ESP_OK &&
            esp_http_client_open(client, (int)length) == ESP_OK && write_parts(client, parts, count, id);
        if (sent && current(id) && esp_http_client_fetch_headers(client) >= 0) {
            result.http_status = esp_http_client_get_status_code(client);
            if (result.http_status == 200) {
                result.client = client;
                if (reused) atomic_store(&http_reused, true);
                break;
            }
        }
        esp_http_client_cleanup(client);
        client = NULL;
        // Retry a stale reused socket only before the full request was transmitted.
        if (!reused || sent || !current(id)) break;
        reused = false;
    }
    memset(authorization, 0, sizeof(authorization));
    return result;
}

static cJSON *read_json(post_t post, uint32_t id, bool keep_connection)
{
    if (!post.client) return NULL;
    char *response = heap_caps_malloc(JSON_CAPACITY, MALLOC_CAP_SPIRAM);
    cJSON *answer = NULL;
    if (response) {
        size_t offset = 0;
        int64_t deadline = esp_timer_get_time() + 12000000;
        while (current(id) && offset < JSON_CAPACITY - 1 && esp_timer_get_time() < deadline) {
            int size = esp_http_client_read(post.client, response + offset, JSON_CAPACITY - 1 - offset);
            if (size < 0) break;
            if (size == 0) {
                if (esp_http_client_is_complete_data_received(post.client)) {
                    response[offset] = 0;
                    answer = cJSON_ParseWithOpts(response, NULL, true);
                }
                break;
            }
            offset += size;
        }
        free(response);
    }
    if (answer && keep_connection && current(id) && esp_http_client_is_persistent_connection(post.client)) {
        asr_connection = post.client;
    } else {
        esp_http_client_cleanup(post.client);
    }
    return answer;
}

static bool transcribe(const utterance_t *job, char *text, size_t size)
{
    static const char preamble[] =
        "--" BOUNDARY "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"speech.wav\"\r\n"
        "Content-Type: audio/wav\r\n\r\n";
    static const char closing[] = "\r\n--" BOUNDARY "--\r\n";
    uint8_t header[44];
    pcm_wav_header(header, (uint32_t)job->bytes, SAMPLE_RATE);
    part_t parts[] = {
        {preamble, sizeof(preamble) - 1}, {header, sizeof(header)},
        {job->data, job->bytes}, {closing, sizeof(closing) - 1},
    };
    post_t post = open_post(ASR_URL, "multipart/form-data; boundary=" BOUNDARY,
                            parts, sizeof(parts) / sizeof(parts[0]), job->id);
    cJSON *body = read_json(post, job->id, job->type == JOB_VOICE);
    const char *message = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(body, "text"));
    text_copy(text, size, message);
    cJSON_Delete(body);
    if (!text[0] && current(job->id)) status("asr_error", post.http_status, NULL, NULL);
    return text[0] != 0;
}

static bool chat(const char *text, const char *expected, uint32_t id, bool include_context,
                 const hardware_context_t *hardware, char *reply, size_t reply_size,
                 char *action, size_t action_size)
{
    uint32_t flash_at = esp_timer_get_time() / 1000;
    char *payload = harness_request(text, expected, include_context, false, hardware);
    if (!payload) return false;
    part_t part = {payload, strlen(payload)};
    post_t post = open_post(FLASH_URL, "application/json", &part, 1, id);
    cJSON_free(payload);
    cJSON *body = read_json(post, id, false);
    record_usage(body);
    cJSON *choices = cJSON_GetObjectItemCaseSensitive(body, "choices");
    cJSON *item = cJSON_GetArrayItem(choices, 0);
    cJSON *message = cJSON_GetObjectItemCaseSensitive(item, "message");
    const char *content = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(message, "content"));
    cJSON *result = content ? cJSON_ParseWithOpts(content, NULL, true) : NULL;
    text_copy(reply, reply_size, cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(result, "reply")));
    text_copy(action, action_size, cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(result, "action")));
    cJSON_Delete(result);
    cJSON_Delete(body);
    bool valid = strcmp(expected, "none") ? action[0] != 0 : reply[0] != 0;
    if (valid) atomic_store(&flash_complete_ms, elapsed_ms(flash_at));
    if (!valid && current(id)) status("flash_error", post.http_status, NULL, NULL);
    return valid;
}

static bool enqueue_pcm(uint32_t id, const int16_t *pcm, size_t samples)
{
    int64_t deadline = esp_timer_get_time() + 1600000;
    while (samples && current(id) && esp_timer_get_time() < deadline) {
        if (audio_push_play(id, pcm, samples * sizeof(int16_t))) return true;
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    return samples == 0;
}

static bool finish_playback(uint32_t id)
{
    audio_end_play(id);
    int64_t deadline = esp_timer_get_time() + 3500000;
    while (esp_timer_get_time() < deadline) {
        if (atomic_load(&playback_done) == id) return true;
        if (atomic_load(&playback_failed) == id || !current(id)) break;
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    return false;
}

static bool speak(const char *reply, uint32_t id)
{
    uint32_t tts_at = esp_timer_get_time() / 1000;
    cJSON *payload = cJSON_CreateObject();
    if (!payload) return false;
    cJSON_AddStringToObject(payload, "text", reply);
    cJSON_AddStringToObject(payload, "voice_id", robot_config.voice);
    cJSON_AddStringToObject(payload, "format", "pcm");
    char *encoded = cJSON_PrintUnformatted(payload);
    cJSON_Delete(payload);
    if (!encoded) return false;
    part_t part = {encoded, strlen(encoded)};
    post_t post = open_post(TTS_URL, "application/json", &part, 1, id);
    cJSON_free(encoded);
    if (!post.client) {
        if (current(id)) status("tts_error", post.http_status, NULL, NULL);
        return false;
    }
    atomic_store(&playback_done, 0);
    atomic_store(&playback_failed, 0);
    bool ok = audio_begin_play(id, false);
    size_t total = 0;
    int read_result = 0;
    bool complete = false;
    const char *failure = "open playback";
    if (ok) {
        status("speaking", 200, NULL, NULL);
        uint8_t input[1536];
        int16_t output[580];
        pcm_resampler_t resampler = {0};
        failure = "receive or queue PCM";
        int64_t deadline = esp_timer_get_time() + 33000000;
        while (ok && current(id) && esp_timer_get_time() < deadline) {
            int bytes = esp_http_client_read(post.client, (char *)input, sizeof(input));
            read_result = bytes;
            if (bytes == 0) break;
            if (bytes < 0 || total + (size_t)bytes > TTS_MAX_BYTES) {
                ok = false;
                break;
            }
            if (!total) atomic_store(&tts_first_ms, elapsed_ms(tts_at));
            total += bytes;
            size_t samples = pcm_resample_24_to_18(&resampler, input, bytes, output,
                                                   sizeof(output) / sizeof(output[0]));
            ok = enqueue_pcm(id, output, samples);
        }
        complete = esp_http_client_is_complete_data_received(post.client);
        ok = ok && current(id) && total >= 6 && !resampler.has_partial && complete;
        if (ok) ok = enqueue_pcm(id, output, pcm_resampler_finish_18(&resampler, output));
    }
    esp_http_client_cleanup(post.client);
    if (ok) {
        failure = "drain playback";
        if (finish_playback(id)) return true;
    }
    if (current(id)) {
        ESP_LOGW(TAG, "TTS failed at %s: HTTP %d, PCM %u bytes, read %d, complete %d",
                 failure, post.http_status, (unsigned)total, read_result, complete);
        status("tts_error", post.http_status, NULL, NULL);
    }
    audio_cancel_turn(id);
    return false;
}

typedef struct {
    esp_websocket_client_handle_t client;
    uint8_t *pcm;
    atomic_uint pcm_bytes;
    atomic_uint first_pcm_at;
    uint32_t open_at, first_text_at;
    atomic_bool connected, failed, finished;
    unsigned opcode;
    size_t notice_bytes;
    char notice[512];
} tts_stream_t;

static void tts_stream_event(void *context, esp_event_base_t base, int32_t event, void *payload)
{
    (void)base;
    tts_stream_t *stream = context;
    if (event == WEBSOCKET_EVENT_CONNECTED) {
        atomic_store(&tts_ws_ms, elapsed_ms(stream->open_at));
        atomic_store(&stream->connected, true);
    } else if (event == WEBSOCKET_EVENT_ERROR || event == WEBSOCKET_EVENT_DISCONNECTED ||
               event == WEBSOCKET_EVENT_CLOSED) {
        if (!atomic_load(&stream->finished)) atomic_store(&stream->failed, true);
    }
    if (event != WEBSOCKET_EVENT_DATA || atomic_load(&stream->failed) ||
        atomic_load(&stream->finished)) return;

    const esp_websocket_event_data_t *data = payload;
    if (data->op_code == 1 || data->op_code == 2) stream->opcode = data->op_code;
    else if (data->op_code != 0) return;
    if (data->data_len < 0) return;
    size_t bytes = data->data_len;
    if (stream->opcode == 2) {
        size_t offset = atomic_load(&stream->pcm_bytes);
        if (bytes > TTS_MAX_BYTES - offset) {
            atomic_store(&stream->failed, true);
        } else if (bytes) {
            memcpy(stream->pcm + offset, data->data_ptr, bytes);
            if (!offset) atomic_store(&stream->first_pcm_at, esp_timer_get_time() / 1000);
            atomic_store(&stream->pcm_bytes, offset + bytes);
        }
    } else if (stream->opcode == 1) {
        if (bytes >= sizeof(stream->notice) - stream->notice_bytes) {
            atomic_store(&stream->failed, true);
            return;
        }
        if (bytes) memcpy(stream->notice + stream->notice_bytes, data->data_ptr, bytes);
        stream->notice_bytes += bytes;
        if (data->fin && data->payload_offset + data->data_len == data->payload_len) {
            stream->notice[stream->notice_bytes] = 0;
            cJSON *message = cJSON_Parse(stream->notice);
            const char *kind = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(message, "event"));
            const char *code = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(message, "code"));
            if (kind && !strcmp(kind, "finish")) {
                if (code && !strcmp(code, "success")) atomic_store(&stream->finished, true);
                else atomic_store(&stream->failed, true);
            }
            cJSON_Delete(message);
            stream->notice_bytes = 0;
        }
    }
}

static void tts_stream_open(tts_stream_t *stream)
{
    stream->open_at = esp_timer_get_time() / 1000;
    char uri[112];
    snprintf(uri, sizeof(uri), TTS_WS_URL "?voice_id=%s&format=pcm", robot_config.voice);
    char headers[sizeof(robot_config.api_key) + 32];
    snprintf(headers, sizeof(headers), "Authorization: Bearer %s\r\n", robot_config.api_key);
    esp_websocket_client_config_t config = {
        .uri = uri, .headers = headers, .crt_bundle_attach = esp_crt_bundle_attach,
        .network_timeout_ms = 5500, .disable_auto_reconnect = true,
        .task_stack = 8192, .buffer_size = 2048,
    };
    stream->pcm = heap_caps_malloc(TTS_MAX_BYTES, MALLOC_CAP_SPIRAM);
    if (stream->pcm) stream->client = esp_websocket_client_init(&config);
    memset(headers, 0, sizeof(headers));
    if (!stream->client ||
        esp_websocket_register_events(stream->client, WEBSOCKET_EVENT_ANY, tts_stream_event, stream) != ESP_OK ||
        esp_websocket_client_start(stream->client) != ESP_OK) atomic_store(&stream->failed, true);
}

static bool tts_stream_send(tts_stream_t *stream, const char *text)
{
    cJSON *body = cJSON_CreateObject();
    if (!body) return false;
    cJSON_AddStringToObject(body, "event", text ? "text" : "finish_text");
    if (text) cJSON_AddStringToObject(body, "text", text);
    char *encoded = cJSON_PrintUnformatted(body);
    cJSON_Delete(body);
    if (!encoded) return false;
    if (text && !stream->first_text_at) stream->first_text_at = esp_timer_get_time() / 1000;
    int bytes = strlen(encoded);
    bool ok = esp_websocket_client_send_text(stream->client, encoded, bytes, pdMS_TO_TICKS(650)) == bytes;
    cJSON_free(encoded);
    return ok;
}

static bool tts_stream_pump(tts_stream_t *stream, uint32_t id, bool *started,
                            size_t *consumed, pcm_resampler_t *resampler)
{
    if (atomic_load(&stream->failed) || atomic_load(&playback_failed) == id || !current(id)) return false;
    size_t available = atomic_load(&stream->pcm_bytes);
    if (available > 0 && !*started) {
        if (!audio_begin_play(id, false)) return false;
        *started = true;
        if (stream->first_text_at) {
            atomic_store(&tts_first_ms, (int)(atomic_load(&stream->first_pcm_at) - stream->first_text_at));
        }
        status("speaking", 200, NULL, NULL);
    }
    while (*consumed < available && current(id)) {
        size_t bytes = available - *consumed;
        if (bytes > 1024) bytes = 1024;
        int16_t output[390];
        pcm_resampler_t next = *resampler;
        size_t count = pcm_resample_24_to_18(&next, stream->pcm + *consumed, bytes,
                                              output, sizeof(output) / sizeof(output[0]));
        if (count && !audio_push_play(id, output, count * sizeof(int16_t))) break;
        *resampler = next;
        *consumed += bytes;
    }
    return !atomic_load(&stream->failed) && atomic_load(&playback_failed) != id && current(id);
}

static bool flush_stream_text(tts_stream_t *stream, const char *reply, size_t *sent, size_t length,
                              bool final)
{
    if (atomic_load(&stream->failed) || !atomic_load(&stream->connected)) return false;
    if (*sent < length) {
        const char *pending = reply + *sent;
        bool boundary = strpbrk(pending, ".?!\n") || strstr(pending, "。") ||
                        strstr(pending, "！") || strstr(pending, "？") || strstr(pending, "，");
        if (final || boundary || length - *sent >= 24) {
            if (!tts_stream_send(stream, pending)) return false;
            *sent = length;
        }
    }
    return !final || tts_stream_send(stream, NULL);
}

static bool flash_stream_event(char *line, char *reply, size_t capacity, size_t *length,
                               bool *done, bool *stopped)
{
    if (strncmp(line, "data:", 5)) return true;
    const char *event = line + 5;
    while (*event == ' ') ++event;
    if (!strcmp(event, "[DONE]")) {
        *done = true;
        return true;
    }
    cJSON *body = cJSON_ParseWithOpts(event, NULL, true);
    if (!body || cJSON_GetObjectItemCaseSensitive(body, "error")) {
        cJSON_Delete(body);
        return false;
    }
    record_usage(body);
    cJSON *choice = cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(body, "choices"), 0);
    const char *finish = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(choice, "finish_reason"));
    const char *content = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(
        cJSON_GetObjectItemCaseSensitive(choice, "delta"), "content"));
    if (content && *length < capacity - 1) {
        text_copy(reply + *length, capacity - *length, content);
        *length += strlen(reply + *length);
    }
    if (finish) *stopped = !strcmp(finish, "stop") || !strcmp(finish, "length");
    bool ok = !finish || *stopped;
    cJSON_Delete(body);
    return ok;
}

static bool stream_chat(const char *text, uint32_t id, bool include_context,
                        const hardware_context_t *hardware, char *reply, size_t capacity)
{
    int64_t start = esp_timer_get_time();
    atomic_store(&flash_first_ms, 0);
    atomic_store(&flash_complete_ms, 0);
    atomic_store(&tts_first_ms, 0);
    atomic_store(&playback_done, 0);
    atomic_store(&playback_failed, 0);
    char *request = harness_request(text, "none", include_context, true, hardware);
    if (!request) return false;
    tts_stream_t stream = {0};
    tts_stream_open(&stream);
    part_t part = {request, strlen(request)};
    post_t post = open_post(FLASH_URL, "application/json", &part, 1, id);
    cJSON_free(request);

    bool started = false, done = false, stopped = false, valid = post.client != NULL;
    size_t length = 0, sent = 0, consumed = 0;
    pcm_resampler_t resampler = {0};
    if (post.client) {
        esp_http_client_set_timeout_ms(post.client, 220);
        char line[2048], input[192];
        size_t used = 0;
        int64_t deadline = esp_timer_get_time() + 14000000;
        while (valid && !done && current(id) && esp_timer_get_time() < deadline) {
            int bytes = esp_http_client_read(post.client, input, sizeof(input));
            if (bytes == 0 || (bytes < 0 && bytes != -ESP_ERR_HTTP_EAGAIN)) break;
            size_t before = length;
            for (int i = 0; i < bytes && valid && !done; ++i) {
                if (input[i] == '\r') continue;
                if (input[i] == '\n') {
                    line[used] = 0;
                    valid = flash_stream_event(line, reply, capacity, &length, &done, &stopped);
                    used = 0;
                } else if (used < sizeof(line) - 1) {
                    line[used++] = input[i];
                } else {
                    valid = false;
                }
            }
            if (length != before) {
                if (!before) atomic_store(&flash_first_ms, (int)((esp_timer_get_time() - start) / 1000));
                status(started ? "speaking" : "flash", 200, NULL, reply);
            }
            if (valid && atomic_load(&stream.connected) && !atomic_load(&stream.failed) &&
                !flush_stream_text(&stream, reply, &sent, length, false)) atomic_store(&stream.failed, true);
            if (!atomic_load(&stream.failed) &&
                !tts_stream_pump(&stream, id, &started, &consumed, &resampler)) {
                atomic_store(&stream.failed, true);
            }
        }
        esp_http_client_cleanup(post.client);
    }
    valid = valid && done && stopped && length && current(id);
    if (valid) atomic_store(&flash_complete_ms, (int)((esp_timer_get_time() - start) / 1000));
    bool spoken = false;
    if (valid) {
        // Flash can finish a short reply while the parallel TLS upgrade is still in flight.
        int64_t connect_deadline = esp_timer_get_time() + 750000;
        while (!atomic_load(&stream.connected) && !atomic_load(&stream.failed) &&
               current(id) && esp_timer_get_time() < connect_deadline) vTaskDelay(pdMS_TO_TICKS(15));
        valid = current(id);
    }
    if (valid && flush_stream_text(&stream, reply, &sent, length, true)) {
        int64_t deadline = esp_timer_get_time() + 35000000;
        while (esp_timer_get_time() < deadline &&
               tts_stream_pump(&stream, id, &started, &consumed, &resampler)) {
            if (atomic_load(&stream.finished) && consumed == atomic_load(&stream.pcm_bytes)) {
                int16_t tail[2];
                spoken = consumed >= 6 && !resampler.has_partial &&
                         enqueue_pcm(id, tail, pcm_resampler_finish_18(&resampler, tail));
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(15));
        }
    }
    if (spoken) audio_end_play(id);
    if (stream.client) esp_websocket_client_destroy(stream.client);
    free(stream.pcm);
    if (spoken && finish_playback(id)) return true;
    if (!current(id)) return false;
    if (!started && valid) {
        ESP_LOGW(TAG, "Streaming TTS unavailable before playback, using HTTP TTS");
        status("tts", 0, NULL, reply);
        return speak(reply, id);
    }
    status(valid ? "tts_error" : "flash_error", post.http_status, NULL, NULL);
    if (started) audio_cancel_turn(id);
    return false;
}

static const char *intent_name(intent_t intent)
{
    switch (intent.type) {
        case INTENT_DRIVE: return action_name(intent.action);
        case INTENT_STOP: return "stop";
        case INTENT_EXPLORE: return "explore";
        case INTENT_GOTO: return "navigate";
        case INTENT_MARK: return "mark";
        case INTENT_REMEMBER: return "remember";
        case INTENT_FACE_SOUND: return "face_sound";
        case INTENT_VOLUME_SET:
        case INTENT_VOLUME_CHANGE:
        case INTENT_VOICE_PACE:
        case INTENT_MOTOR_LOCK:
        case INTENT_MOTOR_ENABLE:
        case INTENT_END_DIALOG:
        case INTENT_CHAT: return "none";
    }
    return "none";
}

static int voice_pwm(voice_pace_t pace)
{
    if (pace == PACE_CURRENT) pace = atomic_load(&voice_pace);
    if (pace == PACE_SLOW) return 82;
    if (pace == PACE_FAST) return 134;
    return 107;
}

static const char *drive_block_reason(action_t action)
{
    motion_status_t chassis = motion_status();
    if (!chassis.armed) return "我的轮子还锁着呢，请大人先帮我开启电机。";
    if (action == FORWARD && chassis.blocked) {
        return chassis.valid ? "前面有东西挡着，我先停在这儿。" : "我还没量清前面有多远，先停一下。";
    }
    return NULL;
}

static const char *try_drive(intent_t intent, uint32_t turn)
{
    if (!nav_cloud_motion(intent.action, voice_pwm(intent.pace), 275, turn)) {
        const char *blocked = drive_block_reason(intent.action);
        return blocked ? blocked : "我先等等，等准备好了我们再试一次。";
    }
    switch (intent.action) {
        case FORWARD: return "好，我往前挪一点。";
        case BACK: return "我往后退一小步。";
        case LEFT: return "我朝左边转一点。";
        case RIGHT: return "好，我朝右边转一点。";
        case WIGGLE: return "看，我来扭一下。";
        case STOP: return NULL;
    }
    return NULL;
}

typedef struct { const char *text; const uint8_t *start, *end; } local_clip_t;
#define LOCAL_CLIP(name, phrase) \
    extern const uint8_t reply_##name##_start[] asm("_binary_reply_" #name "_pcm_start"); \
    extern const uint8_t reply_##name##_end[] asm("_binary_reply_" #name "_pcm_end"); \
    static const local_clip_t name##_clip = {phrase, reply_##name##_start, reply_##name##_end}
LOCAL_CLIP(stop, "我停下啦。");
LOCAL_CLIP(lock, "轮子锁好了。");
LOCAL_CLIP(soft, "我小声一点。");
LOCAL_CLIP(loud, "我大声一点。");
LOCAL_CLIP(volume, "音量调好啦。");
LOCAL_CLIP(slow, "下次慢一点。");
LOCAL_CLIP(fast, "下次快一点。");
LOCAL_CLIP(normal, "恢复正常速度。");
LOCAL_CLIP(bye, "好，下次聊。");
#undef LOCAL_CLIP

static bool speak_clip(const local_clip_t *clip, uint32_t id)
{
    atomic_store(&playback_done, 0);
    atomic_store(&playback_failed, 0);
    if (!audio_begin_play(id, false)) {
        if (current(id)) status("tts_error", 0, NULL, NULL);
        return false;
    }
    for (const uint8_t *pcm = clip->start; pcm < clip->end;) {
        size_t bytes = clip->end - pcm;
        if (bytes > MAX_WIRE_BYTES) bytes = MAX_WIRE_BYTES;
        if (!enqueue_pcm(id, (const int16_t *)pcm, bytes / sizeof(int16_t))) break;
        pcm += bytes;
        if (pcm == clip->end && finish_playback(id)) return true;
    }
    if (current(id)) status("tts_error", 0, NULL, NULL);
    audio_cancel_turn(id);
    return false;
}

static const char *perform_local(intent_t intent, uint32_t id, const local_clip_t **clip)
{
    switch (intent.type) {
        case INTENT_DRIVE:
            return drive_block_reason(intent.action);
        case INTENT_STOP:
            nav_stop(false);
            *clip = &stop_clip;
            break;
        case INTENT_MOTOR_LOCK:
            nav_stop(true);
            *clip = &lock_clip;
            break;
        case INTENT_END_DIALOG:
            audio_end_dialog(id);
            *clip = &bye_clip;
            break;
        case INTENT_MOTOR_ENABLE:
            return motion_status().armed ? "我的轮子已经准备好啦。你想让我往哪边走？" :
                "请大人在我的网页上开启电机。我在这儿等你。";
        case INTENT_VOICE_PACE:
            atomic_store(&voice_pace, intent.pace);
            *clip = intent.pace == PACE_SLOW ? &slow_clip : intent.pace == PACE_FAST ? &fast_clip : &normal_clip;
            break;
        case INTENT_VOLUME_SET:
        case INTENT_VOLUME_CHANGE: {
            int applied;
            if (!network_change_volume(intent.volume, intent.type == INTENT_VOLUME_CHANGE, &applied)) {
                return "我刚才没调好声音，再试一下吧。";
            }
            if (applied == 0) return "好，我先安静一会儿。";
            if (applied == 75 || intent.type == INTENT_VOLUME_SET) *clip = &volume_clip;
            else *clip = intent.volume > 0 ? &loud_clip : &soft_clip;
            break;
        }
        default: return NULL;
    }
    return (*clip)->text;
}

static const char *perform(intent_t intent, direction_t direction, uint32_t turn)
{
    bool accepted;
    switch (intent.type) {
        case INTENT_DRIVE:
            return try_drive(intent, turn);
        case INTENT_EXPLORE:
            return nav_explore_for_turn(turn) ? "好呀，我慢慢看看周围。" : "我还没准备好。请大人帮我准备一下吧。";
        case INTENT_GOTO:
            return nav_go_name_for_turn(intent.argument, turn) ? "好呀，我试着开过去。" :
                "我还没找到路。请大人帮我看看吧。";
        case INTENT_MARK:
            return nav_mark_for_turn(intent.argument, turn) && nav_save() ? "好呀，我记住这个地方了。" :
                "我现在还记不下这个地方。我们等一等吧。";
        case INTENT_REMEMBER:
            return harness_remember(intent.argument) ? "好呀，我记住啦。" : "我现在没记下来。我们等会儿再试吧。";
        case INTENT_FACE_SOUND:
            if (direction != DIR_LEFT && direction != DIR_RIGHT) return "我还没听出你在哪边。可以再叫我一声吗？";
            accepted = nav_cloud_motion(direction == DIR_LEFT ? LEFT : RIGHT, voice_pwm(PACE_CURRENT), 170, turn) != 0;
            return accepted ? "好呀，我试着转过来。" : "我现在还不能转。我们等一等吧。";
        default: return NULL;
    }
}

static void close_asr_connection(void)
{
    if (asr_connection) esp_http_client_cleanup(asr_connection);
    asr_connection = NULL;
}

static void finish_dialog(uint32_t id)
{
    if (current(id)) status("idle", 200, NULL, NULL);
}

static void handle(utterance_t *job)
{
    uint32_t id = job->id;
    if (!current(id)) return;
    char text[243] = {0}, reply[301] = {0}, action[32] = {0};
    atomic_store(&timing_id, 0);
    atomic_store(&asr_ms, 0);
    atomic_store(&first_play_ms, 0);
    atomic_store(&tts_ws_ms, 0);
    atomic_store(&prompt_tokens, -1);
    atomic_store(&completion_tokens, -1);
    atomic_store(&cached_tokens, -1);
    atomic_store(&http_reused, false);
    atomic_store(&timing_origin_ms,
        job->type <= JOB_ASR_ONLY ? job->speech_end_ms : esp_timer_get_time() / 1000);
    atomic_store(&timing_id, id);
    atomic_store(&flash_first_ms, 0);
    atomic_store(&flash_complete_ms, 0);
    atomic_store(&tts_first_ms, 0);
    if (job->type == JOB_TEXT_STREAM) {
        const char *prompt = (const char *)job->data;
        hardware_context_t hardware = snapshot_hardware(DIR_UNKNOWN);
        status("flash", 0, prompt, "");
        if (stream_chat(prompt, id, false, &hardware, reply, sizeof(reply)) && current(id)) {
            status("tts_done", 200, NULL, reply);
        }
        return;
    }
    if (job->type == JOB_TEXT_CHAT) {
        const char *prompt = (const char *)job->data;
        hardware_context_t hardware = snapshot_hardware(DIR_UNKNOWN);
        status("flash", 0, prompt, "");
        if (chat(prompt, "none", id, false, &hardware, reply, sizeof(reply), action, sizeof(action)) && current(id)) {
            status("flash_done", 200, NULL, reply);
        }
        return;
    }
    if (job->type == JOB_TEXT_TTS) {
        const char *prompt = (const char *)job->data;
        status("tts", 0, "", prompt);
        if (speak(prompt, id) && current(id)) status("tts_done", 200, NULL, NULL);
        return;
    }
    status("asr", 0, NULL, NULL);
    uint32_t asr_at = esp_timer_get_time() / 1000;
    bool transcribed = transcribe(job, text, sizeof(text));
    atomic_store(&asr_ms, elapsed_ms(asr_at));
    if (!transcribed || !current(id)) {
        audio_end_dialog(id);
        return;
    }
    if (job->type == JOB_ASR_ONLY) {
        snapshot_hardware(job->direction);
        status("asr_done", 200, text, "");
        return;
    }
    intent_t intent = intent_from_transcript(text, audio_selected_wake());
    const local_clip_t *clip = NULL;
    const char *local = perform_local(intent, id, &clip);
    hardware_context_t hardware = snapshot_hardware(job->direction);
    if (local) {
        close_asr_connection();
        if (!audio_volume()) {
            status("idle", 200, text, local);
            audio_offer_followup(id);
        } else if (clip) {
            status("speaking", 0, text, local);
            if (speak_clip(clip, id)) finish_dialog(id);
        } else {
            status("tts", 0, text, local);
            if (speak(local, id)) finish_dialog(id);
        }
        return;
    }
    if (intent.type == INTENT_CHAT) {
        status("flash", 0, text, "");
        if (stream_chat(text, id, true, &hardware, reply, sizeof(reply))) {
            harness_add_turn(text, reply);
            finish_dialog(id);
        } else {
            audio_end_dialog(id);
        }
        return;
    }
    const char *expected = intent_name(intent);
    bool successful = true;
    status("flash", 0, text, NULL);
    if (!chat(text, expected, id, true, &hardware, reply, sizeof(reply), action, sizeof(action))) {
        if (!current(id)) return;
        successful = false;
        text_copy(reply, sizeof(reply), "我听到啦。等一会儿，我们再聊吧。");
    }
    if (!current(id)) return;
    if (successful) {
        const char *outcome;
        if (esp_timer_get_time() - job->captured_at > 12000000) {
            outcome = "等得有点久。你可以再说一遍吗？";
        } else if (strcmp(action, expected)) {
            outcome = "我还没听明白。你可以慢慢再说一遍吗？";
        } else {
            outcome = perform(intent, job->direction, id);
        }
        if (outcome) text_copy(reply, sizeof(reply), outcome);
    }
    if (!current(id)) return;
    status("tts", 0, text, reply);
    if (speak(reply, id)) {
        if (successful) harness_add_turn(text, reply);
        finish_dialog(id);
    }
}

static void cloud_task(void *unused)
{
    (void)unused;
    while (true) {
        utterance_t *job;
        if (xQueueReceive(jobs, &job, portMAX_DELAY) == pdTRUE) {
            handle(job);
            close_asr_connection();
            release_job(job);
        }
    }
}

void cloud_start(void)
{
    status_lock = xSemaphoreCreateMutex();
    jobs = xQueueCreate(1, sizeof(utterance_t *));
    assert(status_lock && jobs);
    if (xTaskCreate(cloud_task, "deepseek", 11776, NULL, 4, NULL) == pdPASS) {
        atomic_store(&initialized, true);
    } else {
        ESP_LOGE(TAG, "Cloud worker unavailable");
    }
}
