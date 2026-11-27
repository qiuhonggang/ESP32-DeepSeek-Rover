#pragma once
#include <stdbool.h>
#include <stddef.h>

enum { TTS_SEGMENT_CHARS = 24, TTS_SEGMENT_BYTES = TTS_SEGMENT_CHARS * 4 };

// Return the byte length of the next UTF-8 segment ready for synthesis.
size_t tts_text_segment(const char *text, size_t bytes, bool final);
