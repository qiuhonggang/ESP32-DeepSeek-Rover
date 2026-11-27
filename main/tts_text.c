#include "tts_text.h"
#include <ctype.h>
#include <string.h>

size_t tts_text_segment(const char *text, size_t bytes, bool final)
{
    size_t offset = 0, pause = 0, space = 0;
    for (unsigned chars = 1; offset < bytes; ++chars) {
        const char *p = text + offset;
        unsigned char c = (unsigned char)*p;
        size_t width = 1;
        if (c >= 0xf0) width = 4;
        else if (c >= 0xe0) width = 3;
        else if (c >= 0xc0) width = 2;
        if (width > bytes - offset) break;
        offset += width;

        bool decimal = c == '.' && p > text && isdigit((unsigned char)p[-1]) &&
                       (offset < bytes ? isdigit((unsigned char)text[offset]) : !final);
        if ((c == '.' && !decimal) || c == '!' || c == '?' || c == ';' || c == '\n' ||
            (width == 3 && (!memcmp(p, "。", 3) || !memcmp(p, "！", 3) ||
                            !memcmp(p, "？", 3) || !memcmp(p, "；", 3)))) return offset;

        if (chars >= 6 && (c == ',' ||
            (width == 3 && (!memcmp(p, "，", 3) || !memcmp(p, "、", 3))))) pause = offset;
        if (chars >= 6 && c == ' ') space = offset;
        if (chars == TTS_SEGMENT_CHARS) {
            if (pause) return pause;
            if (space) return space;
            return decimal ? offset - 2 : offset;
        }
    }
    return final ? offset : pause;
}
