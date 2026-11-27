#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

// One cloud turn owns this state and the published text until done is set.
typedef struct {
    const char *text;
    uint32_t id;
    atomic_uint available;
    atomic_bool final, cancelled, done, started;
    atomic_int first_ms;
    bool success;
} local_tts_t;

bool local_tts_start(local_tts_t *stream, uint32_t id, const char *text);
void local_tts_publish(local_tts_t *stream, size_t bytes, bool final);
