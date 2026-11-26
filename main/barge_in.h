#pragma once
#include <stdbool.h>

enum {
    BARGE_VOICE_START_MS = 64,
    BARGE_VOICE_CONFIRM_MS = 96,
    BARGE_ECHO_SETTLE_MS = 160,
    BARGE_RENDER_QUIET_MS = 96,
    BARGE_CHECK_LIMIT_MS = 416,
    BARGE_ECHO_LEARN_MS = 128,
};

typedef struct {
    int echo_rms, started_ms, candidate_ms, candidate_peak;
    int silence_ms, render_quiet_ms, confirmed_ms, retry_ms;
} barge_in_t;

static inline void barge_in_reference(barge_in_t *state, int rms, int block_ms)
{
    state->render_quiet_ms = state->silence_ms && rms < 12 ? state->render_quiet_ms + block_ms : 0;
}

static inline bool barge_in_near_mic(const barge_in_t *state)
{
    return state->silence_ms > BARGE_ECHO_SETTLE_MS && state->render_quiet_ms >= BARGE_RENDER_QUIET_MS;
}

static inline bool barge_in_step(barge_in_t *state, int rms, bool speech, int threshold, int block_ms)
{
    if (state->silence_ms) {
        state->confirmed_ms = barge_in_near_mic(state) && speech && rms >= threshold ?
                              state->confirmed_ms + block_ms : 0;
        if (state->confirmed_ms >= BARGE_VOICE_CONFIRM_MS) return true;
        state->silence_ms += block_ms;
        if (state->silence_ms <= BARGE_CHECK_LIMIT_MS) return false;
        state->silence_ms = state->render_quiet_ms = state->confirmed_ms = state->candidate_ms = 0;
        if (state->candidate_peak > state->echo_rms) state->echo_rms = state->candidate_peak;
        state->candidate_peak = 0;
        state->retry_ms = 384;
        return false;
    }

    // Learn residual playback energy between speech candidates. A transient
    // alone must never end playback. Confirm it with the speaker paused.
    int echo_margin = state->echo_rms / 2 + threshold / 8;
    bool candidate = state->started_ms >= BARGE_ECHO_LEARN_MS && state->retry_ms <= 0 &&
                     speech && rms >= threshold && rms >= state->echo_rms + echo_margin;
    if (candidate) {
        state->candidate_ms += block_ms;
        if (rms > state->candidate_peak) state->candidate_peak = rms;
        if (state->candidate_ms >= BARGE_VOICE_START_MS) state->silence_ms = block_ms;
    } else {
        if (state->started_ms < BARGE_ECHO_LEARN_MS) state->started_ms += block_ms;
        state->retry_ms -= state->retry_ms > 0 ? block_ms : 0;
        state->echo_rms = (state->echo_rms * 7 + rms) / 8;
        state->candidate_ms = state->candidate_peak = 0;
    }
    return false;
}
