#include "audio_wire.h"
#include <string.h>

static void le16(uint8_t *at, uint16_t value)
{
    at[0] = value & 255;
    at[1] = value >> 8;
}
static void le32(uint8_t *at, uint32_t value)
{
    le16(at, value & 65535);
    le16(at + 2, value >> 16);
}

void pcm_wav_header(uint8_t header[44], uint32_t pcm_bytes, uint32_t rate)
{
    memcpy(header, "RIFF", 4);
    le32(header + 4, 36 + pcm_bytes);
    memcpy(header + 8, "WAVEfmt ", 8);
    le32(header + 16, 16);
    le16(header + 20, 1);
    le16(header + 22, 1);
    le32(header + 24, rate);
    le32(header + 28, rate * 2);
    le16(header + 32, 2);
    le16(header + 34, 16);
    memcpy(header + 36, "data", 4);
    le32(header + 40, pcm_bytes);
}

// A small 3-to-2 box resampler. It averages across each 1.5-sample output interval,
// retaining partial input across TLS reads for continuous playback at the I2S rate.
static size_t resample(pcm_resampler_t *state, const uint8_t *in, size_t bytes,
                       int16_t *out, size_t capacity, unsigned group)
{
    if (capacity < ((state->count + (bytes + state->has_partial) / 2) / group) * (group - 1)) return 0;
    size_t written = 0;
    for (size_t i = 0; i < bytes; ++i) {
        if (!state->has_partial) {
            state->partial = in[i];
            state->has_partial = 1;
            continue;
        }
        state->group[state->count++] = (int16_t)((uint16_t)state->partial | ((uint16_t)in[i] << 8));
        state->has_partial = 0;
        if (state->count == group) {
            for (unsigned j = 0; j + 1 < group; ++j) {
                out[written++] = ((int32_t)(group - 1 - j) * state->group[j] +
                                  (int32_t)(j + 1) * state->group[j + 1]) / (int32_t)group;
            }
            state->count = 0;
        }
    }
    return written;
}

size_t pcm_resample_24_to_16(pcm_resampler_t *state, const uint8_t *in, size_t bytes,
                             int16_t *out, size_t capacity)
{
    return resample(state, in, bytes, out, capacity, 3);
}

size_t pcm_resample_24_to_18(pcm_resampler_t *state, const uint8_t *in, size_t bytes,
                             int16_t *out, size_t capacity)
{
    return resample(state, in, bytes, out, capacity, 4);
}

size_t pcm_resampler_finish(pcm_resampler_t *state, int16_t out[2])
{
    if (!state->count) return 0;
    out[0] = state->group[0];
    if (state->count == 2) out[0] = (int16_t)((2 * (int32_t)state->group[0] + state->group[1]) / 3);
    state->count = 0;
    state->has_partial = 0;
    return 1;
}

size_t pcm_resampler_finish_18(pcm_resampler_t *state, int16_t out[2])
{
    if (!state->count) return 0;
    out[0] = state->count == 1 ? state->group[0] :
        (3 * (int32_t)state->group[0] + state->group[1]) / 4;
    if (state->count == 3) out[1] = ((int32_t)state->group[1] + state->group[2]) / 2;
    size_t written = state->count == 3 ? 2 : 1;
    state->count = 0;
    state->has_partial = 0;
    return written;
}
