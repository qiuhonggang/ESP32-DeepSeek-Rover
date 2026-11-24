#include "intent.h"
#include <ctype.h>
#include <string.h>

void text_copy(char *target, size_t length, const char *source)
{
    if (length == 0) return;
    if (!source) source = "";
    size_t count = strnlen(source, length);
    if (count == length) {
        count = length - 1;
        while (count && ((unsigned char)source[count] & 0xc0) == 0x80) --count;
    }
    memcpy(target, source, count);
    target[count] = 0;
}

static size_t punctuation(const char *text)
{
    const char *symbols[] = {"。", "，", "、", "！", "？", "；", "：", "　"};
    for (int i = 0; i < sizeof(symbols) / sizeof(symbols[0]); ++i) {
        if (!strncmp(text, symbols[i], 3)) return 3;
    }
    unsigned char c = (unsigned char)*text;
    return c < 128 && (isspace(c) || ispunct(c)) ? 1 : 0;
}

static bool in(const char *value, const char *const *choices)
{
    for (int i = 0; choices[i]; ++i) {
        if (!strcmp(value, choices[i])) return true;
    }
    return false;
}

static const char *consume(const char *text, const char *prefix)
{
    size_t length = strlen(prefix);
    return strncmp(text, prefix, length) == 0 ? text + length : NULL;
}

static size_t pause_separator(const char *text)
{
    if (*text == ',') return 1;
    return !strncmp(text, "，", 3) || !strncmp(text, "、", 3) ? 3 : 0;
}

static const char *match_phrase(const char *text, const char *phrase)
{
    if (!phrase || !*phrase) return NULL;
    bool started = false;
    size_t pause;
    while (*phrase) {
        while ((pause = pause_separator(phrase)) != 0) phrase += pause;
        if (!*phrase) break;
        if (started) while ((pause = pause_separator(text)) != 0) text += pause;
        unsigned char p = (unsigned char)*phrase++;
        unsigned char t = (unsigned char)*text;
        if (!t || (t < 128 ? tolower(t) : t) != (p < 128 ? tolower(p) : p)) return NULL;
        ++text;
        started = true;
    }
    return started ? text : NULL;
}

static const char *drop_wake_prefix(const char *text, const wake_option_t *option)
{
    if (!option) return text;
    const char *after = NULL;
    // Some ASR spellings extend the display name (e.g. 星宝儿), so try aliases first.
    for (size_t i = 0; i < sizeof(option->aliases) / sizeof(option->aliases[0]); ++i) {
        after = match_phrase(text, option->aliases[i]);
        if (after) break;
    }
    if (!after) after = match_phrase(text, option->phrase);
    if (!after) return text;
    size_t skip;
    while ((skip = pause_separator(after)) != 0) after += skip;
    if (!strncmp(after, "。", 3) || !strncmp(after, "！", 3)) after += 3;
    else if (*after == '.' || *after == '!') ++after;
    while ((skip = pause_separator(after)) != 0) after += skip;
    return after;
}

static bool short_suffix(const char *suffix)
{
    const char *const allowed[] = {"", "吧", "一下", "一点", "一点点", "一步", "一点吧", "一下吧", NULL};
    return in(suffix, allowed);
}

intent_t intent_from_transcript(const char *transcript, const wake_option_t *wake_word)
{
    intent_t result = {.type = INTENT_CHAT, .action = STOP};
    if (!transcript) return result;

    // Preserve punctuation boundaries inside a sentence. Composite commands and
    // quoted/descriptive sentences stay chat unless the whole phrase is supported.
    char normalized[304];
    size_t count = 0;
    while (*transcript && count < sizeof(normalized) - 1) {
        unsigned char c = (unsigned char)*transcript++;
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
        normalized[count++] = c < 128 ? (char)tolower(c) : (char)c;
    }
    normalized[count] = 0;
    while (count && punctuation(normalized + count - 1) == 1) normalized[--count] = 0;
    while (count >= 3 && punctuation(normalized + count - 3) == 3) {
        count -= 3;
        normalized[count] = 0;
    }

    const char *text = drop_wake_prefix(normalized, wake_word);
    for (int i = 0; i < 4; ++i) {
        const char *next = consume(text, "请");
        if (!next) next = consume(text, "小车");
        if (!next) next = consume(text, "你");
        if (!next) break;
        text = next;
        size_t skip;
        while ((skip = pause_separator(text)) != 0) text += skip;
    }

    const char *const stop[] = {"停", "停下", "停下来", "停止", "停止巡游", "停止导航",
                                "急停", "别动", "不要动", "停车", "停下吧", NULL};
    if (in(text, stop)) { result.type = INTENT_STOP; return result; }

    const char *const explore[] = {"开始巡游", "自动巡游", "开始建图", "开始探索", "建图", NULL};
    if (in(text, explore)) { result.type = INTENT_EXPLORE; return result; }

    const char *const home[] = {"返回起点", "回起点", "回到起点", NULL};
    if (in(text, home)) {
        result.type = INTENT_GOTO;
        text_copy(result.argument, sizeof(result.argument), "起点");
        return result;
    }

    const char *const face[] = {"朝我转", "朝我转一下", "转向我", "面向我", "听声转向", NULL};
    if (in(text, face)) { result.type = INTENT_FACE_SOUND; return result; }

    const struct { const char *prefix; action_t action; } moves[] = {
        {"前进", FORWARD}, {"往前走", FORWARD}, {"向前走", FORWARD},
        {"后退", BACK}, {"往后走", BACK}, {"向后走", BACK},
        {"左转", LEFT}, {"向左转", LEFT}, {"往左转", LEFT},
        {"右转", RIGHT}, {"向右转", RIGHT}, {"往右转", RIGHT},
        {"扭一扭", WIGGLE}, {"扭一下", WIGGLE}, {"摆一摆", WIGGLE},
    };
    for (int i = 0; i < sizeof(moves) / sizeof(moves[0]); ++i) {
        const char *rest = consume(text, moves[i].prefix);
        if (rest && short_suffix(rest)) {
            result.type = INTENT_DRIVE;
            result.action = moves[i].action;
            return result;
        }
    }

    const char *name = consume(text, "把这里记为");
    if (!name) name = consume(text, "标记这里为");
    if (name && *name && strlen(name) <= 36) result.type = INTENT_MARK;
    else {
        name = consume(text, "导航到");
        if (!name) name = consume(text, "开到");
        if (!name) name = consume(text, "去");
        if (name && *name && strlen(name) <= 36) result.type = INTENT_GOTO;
    }
    if (result.type != INTENT_CHAT) {
        text_copy(result.argument, sizeof(result.argument), name);
        return result;
    }
    name = consume(text, "记住");
    if (name && *name) {
        result.type = INTENT_REMEMBER;
        text_copy(result.argument, sizeof(result.argument), name);
    }
    return result;
}
