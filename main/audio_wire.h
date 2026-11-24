#pragma once
#include <stddef.h>
#include <stdint.h>

typedef struct {
    unsigned char partial;
    unsigned char has_partial;
    unsigned count;
    int16_t group[4];
} pcm_resampler_t;

void pcm_wav_header(uint8_t header[44], uint32_t pcm_bytes, uint32_t rate);
size_t pcm_resample_24_to_16(pcm_resampler_t *state, const uint8_t *in, size_t bytes,
                             int16_t *out, size_t capacity);
size_t pcm_resampler_finish(pcm_resampler_t *state, int16_t out[2]);
size_t pcm_resample_24_to_18(pcm_resampler_t *state, const uint8_t *in, size_t bytes,
                             int16_t *out, size_t capacity);
size_t pcm_resampler_finish_18(pcm_resampler_t *state, int16_t out[2]);
