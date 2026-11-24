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
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define ASR_URL "https://api.deepseek.com/v1/tools/asr"
#define FLASH_URL "https://api.deepseek.com/chat/completions"
#define TTS_URL "https://api.deepseek.com/v1/tools/tts"
#define MAX_RECORD_BYTES (SAMPLE_RATE * 2 * 12)
#define JSON_CAPACITY 8192
#define TTS_MAX_BYTES (24000 * 2 * 25)
#define BOUNDARY "ESP32Rover-a77b2f2d16b64050"
static const char *TAG = "cloud";

typedef struct {
    uint32_t id;
    size_t bytes;
    int64_t captured_at;
    direction_t direction;
    uint8_t data[];
} utterance_t;

typedef struct { const void *data; size_t length; } part_t;
typedef struct { esp_http_client_handle_t client; int http_status; } post_t;

static utterance_t *recording;
static QueueHandle_t jobs;
static SemaphoreHandle_t status_lock;
static char phase[32] = "idle", last_text[243], last_reply[301];
static int last_http;
static atomic_uint playback_done, playback_failed;
static atomic_bool initialized;

static void status(const char *state, int http, const char *text, const char *answer)
{
    xSemaphoreTake(status_lock, portMAX_DELAY);
    strlcpy(phase, state, sizeof(phase));
    last_http = http;
    if (text) text_copy(last_text, sizeof(last_text), text);
    if (answer) text_copy(last_reply, sizeof(last_reply), answer);
    xSemaphoreGive(status_lock);
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
    xSemaphoreGive(status_lock);
    return json;
}

bool cloud_uplink(const uplink_t *packet)
{
    if (!atomic_load(&initialized)) return false;
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
        recording->direction = packet->direction;
        recording->bytes = 0;
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
        if (new_job->bytes < 4800 || !network_ready()) {
            free(new_job);
            status("audio_abort", 0, NULL, NULL);
            return false;
        }
        if (xQueueSend(jobs, &new_job, 0) == pdTRUE) return true;
        utterance_t *stale;
        if (xQueueReceive(jobs, &stale, 0) == pdTRUE) free(stale);
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
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return result;
    char authorization[sizeof(robot_config.api_key) + 8];
    snprintf(authorization, sizeof(authorization), "Bearer %s", robot_config.api_key);
    esp_err_t auth = esp_http_client_set_header(client, "Authorization", authorization);
    memset(authorization, 0, sizeof(authorization));
    if (auth == ESP_OK && esp_http_client_set_header(client, "Content-Type", type) == ESP_OK &&
        esp_http_client_open(client, (int)length) == ESP_OK &&
        write_parts(client, parts, count, id) && current(id) &&
        esp_http_client_fetch_headers(client) >= 0) {
        result.http_status = esp_http_client_get_status_code(client);
        if (result.http_status == 200) {
            result.client = client;
            return result;
        }
    }
    esp_http_client_cleanup(client);
    return result;
}

static cJSON *read_json(post_t post, uint32_t id)
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
    esp_http_client_cleanup(post.client);
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
    cJSON *body = read_json(post, job->id);
    const char *message = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(body, "text"));
    text_copy(text, size, message);
    cJSON_Delete(body);
    if (!text[0] && current(job->id)) status("asr_error", post.http_status, NULL, NULL);
    return text[0] != 0;
}

static bool chat(const char *text, const char *expected, uint32_t id,
                 char *reply, size_t reply_size, char *action, size_t action_size)
{
    char *payload = harness_request(text, expected);
    if (!payload) return false;
    part_t part = {payload, strlen(payload)};
    post_t post = open_post(FLASH_URL, "application/json", &part, 1, id);
    cJSON_free(payload);
    cJSON *body = read_json(post, id);
    cJSON *choices = cJSON_GetObjectItemCaseSensitive(body, "choices");
    cJSON *item = cJSON_GetArrayItem(choices, 0);
    cJSON *message = cJSON_GetObjectItemCaseSensitive(item, "message");
    const char *content = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(message, "content"));
    cJSON *result = content ? cJSON_ParseWithOpts(content, NULL, true) : NULL;
    text_copy(reply, reply_size, cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(result, "reply")));
    text_copy(action, action_size, cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(result, "action")));
    cJSON_Delete(result);
    cJSON_Delete(body);
    if (!reply[0] && current(id)) status("flash_error", post.http_status, NULL, NULL);
    return reply[0] != 0;
}

static bool enqueue_pcm(uint32_t id, const int16_t *pcm, size_t samples)
{
    int64_t deadline = esp_timer_get_time() + 1600000;
    while (samples && current(id) && esp_timer_get_time() < deadline) {
        if (audio_push_play(pcm, samples * sizeof(int16_t))) return true;
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    return samples == 0;
}

static bool speak(const char *reply, uint32_t id)
{
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
    bool ok = audio_begin_play(id);
    if (ok) {
        status("speaking", 200, NULL, NULL);
        uint8_t input[1536];
        int16_t output[520];
        pcm_resampler_t resampler = {0};
        size_t total = 0;
        int64_t deadline = esp_timer_get_time() + 33000000;
        while (ok && current(id) && esp_timer_get_time() < deadline) {
            int bytes = esp_http_client_read(post.client, (char *)input, sizeof(input));
            if (bytes == 0) break;
            if (bytes < 0 || total + (size_t)bytes > TTS_MAX_BYTES) {
                ok = false;
                break;
            }
            total += bytes;
            size_t samples = pcm_resample_24_to_16(&resampler, input, bytes, output,
                                                   sizeof(output) / sizeof(output[0]));
            ok = enqueue_pcm(id, output, samples);
        }
        ok = ok && current(id) && total >= 6 && !resampler.has_partial &&
             esp_http_client_is_complete_data_received(post.client);
        if (ok) ok = enqueue_pcm(id, output, pcm_resampler_finish(&resampler, output));
    }
    esp_http_client_cleanup(post.client);
    if (ok) {
        audio_end_play(id);
        int64_t deadline = esp_timer_get_time() + 3500000;
        while (current(id) && esp_timer_get_time() < deadline) {
            if (atomic_load(&playback_done) == id) return true;
            if (atomic_load(&playback_failed) == id) break;
            vTaskDelay(pdMS_TO_TICKS(25));
        }
    }
    if (current(id)) status("tts_error", post.http_status, NULL, NULL);
    audio_cancel_turn(id);
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
        case INTENT_CHAT: return "none";
    }
    return "none";
}

static const char *perform(intent_t intent, direction_t direction, uint32_t turn)
{
    bool accepted;
    switch (intent.type) {
        case INTENT_DRIVE:
            accepted = nav_cloud_motion(intent.action, 107, 275, turn) != 0;
            return accepted ? "好的。" : "我现在还不能动。请大人帮我看看吧。";
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
            accepted = nav_cloud_motion(direction == DIR_LEFT ? LEFT : RIGHT, 102, 170, turn) != 0;
            return accepted ? "好呀，我试着转过来。" : "我现在还不能转。我们等一等吧。";
        case INTENT_STOP:
        case INTENT_CHAT:
            return NULL;
    }
    return NULL;
}

static void handle(utterance_t *job)
{
    uint32_t id = job->id;
    if (!current(id)) return;
    char text[243] = {0}, reply[301] = {0}, action[32] = {0};
    status("asr", 0, NULL, NULL);
    if (!transcribe(job, text, sizeof(text)) || !current(id)) return;
    intent_t intent = intent_from_transcript(text, audio_selected_wake());
    const char *expected = intent_name(intent);
    bool successful = true;
    if (intent.type == INTENT_STOP) {
        nav_stop(false);
        text_copy(reply, sizeof(reply), "好，我停下啦。");
    } else {
        status("flash", 0, text, NULL);
        if (!chat(text, expected, id, reply, sizeof(reply), action, sizeof(action))) {
            if (!current(id)) return;
            successful = false;
            text_copy(reply, sizeof(reply), "我听到啦。等一会儿，我们再聊吧。");
        }
        if (!current(id)) return;
        if (intent.type != INTENT_CHAT && successful) {
            const char *outcome;
            if (esp_timer_get_time() - job->captured_at > 12000000) {
                outcome = "等得有点久。你可以再说一遍吗？";
            } else if (strcmp(action, expected)) {
                outcome = "我还没听明白。你可以慢慢再说一遍吗？";
            } else {
                outcome = perform(intent, job->direction, id);
            }
            if (outcome) text_copy(reply, sizeof(reply), outcome);
        } else if (intent.type == INTENT_CHAT && action[0] && strcmp(action, "none")) {
            text_copy(reply, sizeof(reply), "你可以跟我说，左转，或者停下。");
        }
    }
    if (!current(id)) return;
    status("tts", 0, text, reply);
    if (speak(reply, id)) {
        if (successful && current(id)) harness_add_turn(text, reply);
        if (current(id)) status("idle", 200, NULL, NULL);
    }
}

static void cloud_task(void *unused)
{
    (void)unused;
    while (true) {
        utterance_t *job;
        if (xQueueReceive(jobs, &job, portMAX_DELAY) == pdTRUE) {
            handle(job);
            free(job);
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
