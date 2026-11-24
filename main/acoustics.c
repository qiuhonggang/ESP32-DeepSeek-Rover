#include "robot.h"
#include <math.h>

int pcm_rms(const int16_t *pcm, size_t count)
{
    if (!count) return 0;
    int64_t power = 0;
    for (size_t i = 0; i < count; ++i) power += (int32_t)pcm[i] * pcm[i];
    return (int)sqrtf((float)power / count);
}

direction_t estimate_direction(const int16_t *left, const int16_t *right, int count, int spacing_mm)
{
    // Positive lag means the sound reached the left microphone first.
    int max_lag = (spacing_mm * SAMPLE_RATE + 342999) / 343000;
    if (max_lag < 1 || count <= 2 * max_lag + 2) return DIR_UNKNOWN;
    float peak = -1.0f;
    int peak_lag = 0;
    for (int lag = -max_lag; lag <= max_lag; ++lag) {
        float cross = 0, left_power = 0, right_power = 0;
        for (int i = max_lag + 1; i < count - max_lag; ++i) {
            int l = left[i] - left[i - 1];
            int r = right[i + lag] - right[i + lag - 1];
            cross += (float)l * r;
            left_power += (float)l * l;
            right_power += (float)r * r;
        }
        if (left_power < 10000 || right_power < 10000) continue;
        float score = cross / sqrtf(left_power * right_power);
        if (score > peak) {
            peak = score;
            peak_lag = lag;
        }
    }
    if (peak < 0.42f) return DIR_UNKNOWN;
    if (peak_lag > 0) return DIR_LEFT;
    if (peak_lag < 0) return DIR_RIGHT;
    return DIR_CENTER;
}
