#pragma once
#include <stdbool.h>
#include <stdint.h>

enum { DIALOG_WAIT_MS = 8000, DIALOG_TRIGGER_MS = 96 };

typedef struct {
    int64_t opens_us, expires_us;
    int voiced_ms;
} dialog_window_t;

static inline bool dialog_wait_step(dialog_window_t *window, int64_t now_us, bool eligible,
                                    bool voiced, int block_ms)
{
    if (!window->expires_us || now_us >= window->expires_us) {
        *window = (dialog_window_t){0};
        return false;
    }
    window->voiced_ms = now_us >= window->opens_us && eligible && voiced ? window->voiced_ms + block_ms : 0;
    if (window->voiced_ms < DIALOG_TRIGGER_MS) return false;
    *window = (dialog_window_t){0};
    return true;
}
