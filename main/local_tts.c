#include "local_tts.h"
#include "robot.h"
#include "tts_text.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "esp_tts_voice_xiaole.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "local_tts";
enum { XIAOLE_SPEED = 2 };

static bool active(const local_tts_t *stream)
{
    return !atomic_load(&stream->cancelled) && audio_turn_is_current(stream->id);
}

static bool play_segment(local_tts_t *stream, esp_tts_handle_t engine, const char *text,
                         int64_t start)
{
    if (!esp_tts_parse_chinese(engine, text)) {
        esp_tts_stream_reset(engine);
        return true;
    }
    while (active(stream)) {
        int count = 0;
        const int16_t *pcm = esp_tts_stream_play(engine, &count, XIAOLE_SPEED);
        if (!count) {
            esp_tts_stream_reset(engine);
            return true;
        }
        if (!pcm || count < 0) break;
        if (!atomic_load(&stream->started)) {
            if (!audio_begin_local_play(stream->id)) break;
            int first_ms = (esp_timer_get_time() - start + 999) / 1000;
            atomic_store(&stream->first_ms, first_ms);
            atomic_store(&stream->started, true);
        }
        for (int offset = 0; offset < count;) {
            size_t bytes = (count - offset) * sizeof(*pcm);
            if (bytes > MAX_WIRE_BYTES) bytes = MAX_WIRE_BYTES;
            int64_t deadline = esp_timer_get_time() + 1600000;
            bool queued = false;
            while (active(stream) && esp_timer_get_time() < deadline) {
                if ((queued = audio_push_play(stream->id, pcm + offset, bytes))) break;
                vTaskDelay(1);
            }
            if (!queued) return false;
            offset += bytes / sizeof(*pcm);
        }
    }
    return false;
}

static void synthesize(void *context)
{
    local_tts_t *stream = context;
    int64_t start = esp_timer_get_time();
    const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_DATA_FAT, "voice_data");
    const void *data = NULL;
    esp_partition_mmap_handle_t mapping = 0;
    esp_tts_voice_t *voice = NULL;
    esp_tts_handle_t engine = NULL;
    if (part && part->size >= XIAOLE_VOICE_BYTES &&
        esp_partition_mmap(part, 0, XIAOLE_VOICE_BYTES, ESP_PARTITION_MMAP_DATA,
                           &data, &mapping) == ESP_OK && !memcmp(data, "xiaole_", 7)) {
        voice = esp_tts_voice_set_init(&esp_tts_voice_xiaole, (void *)data);
        if (voice) engine = esp_tts_create(voice);
    }
    if (!engine) ESP_LOGW(TAG, "Local TTS initialization failed; check the voice_data image");

    size_t offset = 0;
    while (engine && active(stream)) {
        bool final = atomic_load(&stream->final);
        size_t available = atomic_load(&stream->available);
        size_t bytes = tts_text_segment(stream->text + offset, available - offset, final);
        if (!bytes) {
            if (final) {
                stream->success = offset == available && atomic_load(&stream->started);
                break;
            }
            vTaskDelay(1);
            continue;
        }
        char segment[TTS_SEGMENT_BYTES + 1];
        memcpy(segment, stream->text + offset, bytes);
        segment[bytes] = 0;
        if (!play_segment(stream, engine, segment, start)) break;
        offset += bytes;
    }
    if (engine) esp_tts_destroy(engine);
    if (voice) esp_tts_voice_set_free(voice);
    if (data) esp_partition_munmap(mapping);
    atomic_store(&stream->done, true);
    vTaskDelete(NULL);
}
bool local_tts_start(local_tts_t *stream, uint32_t id, const char *text)
{
    *stream = (local_tts_t){.text = text, .id = id};
    return xTaskCreate(synthesize, "local_tts", 6144, stream, 4, NULL) == pdPASS;
}

void local_tts_publish(local_tts_t *stream, size_t bytes, bool final)
{
    atomic_store(&stream->available, bytes);
    if (final) atomic_store(&stream->final, true);
}
