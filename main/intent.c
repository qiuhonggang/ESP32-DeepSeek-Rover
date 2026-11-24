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

static const struct { const char *phrase; voice_pace_t pace; } pace_words[] = {
    {"慢一点", PACE_SLOW}, {"慢一些", PACE_SLOW}, {"慢点", PACE_SLOW}, {"慢慢", PACE_SLOW},
    {"慢速", PACE_SLOW}, {"快一点", PACE_FAST}, {"快一些", PACE_FAST}, {"快点", PACE_FAST},
    {"快速", PACE_FAST}, {"正常速度", PACE_NORMAL}, {"恢复正常速度", PACE_NORMAL},
    {"减速", PACE_SLOW}, {"降速", PACE_SLOW}, {"加速", PACE_FAST}, {"提速", PACE_FAST},
};

static voice_pace_t pace_word(const char *text, const char **rest)
{
    for (size_t i = 0; i < sizeof(pace_words) / sizeof(pace_words[0]); ++i) {
        const char *end = consume(text, pace_words[i].phrase);
        if (end) {
            *rest = end;
            return pace_words[i].pace;
        }
    }
    *rest = text;
    return PACE_CURRENT;
}

static int chinese_digit(const char **text)
{
    static const char *const digits[] = {"零", "一", "二", "三", "四", "五", "六", "七", "八", "九"};
    for (int i = 0; i < 10; ++i) {
        const char *after = consume(*text, digits[i]);
        if (after) {
            *text = after;
            return i;
        }
    }
    return -1;
}

static int volume_number(const char *text)
{
    const char *prefix = consume(text, "百分之");
    if (prefix) text = prefix;
    int value = -1;
    if (*text >= '0' && *text <= '9') {
        value = 0;
        while (*text >= '0' && *text <= '9' && value <= 100) value = value * 10 + *text++ - '0';
    } else if (consume(text, "一百")) {
        value = 100;
        text += strlen("一百");
    } else {
        value = chinese_digit(&text);
        const char *tens = consume(text, "十");
        if (tens) {
            text = tens;
            int units = chinese_digit(&text);
            value = (value < 0 ? 10 : value * 10) + (units < 0 ? 0 : units);
        }
    }
    const char *percent = consume(text, "%");
    if (!percent) percent = consume(text, "％");
    if (percent) text = percent;
    return value >= 0 && value <= 100 && (!*text || !strcmp(text, "吧")) ? value : -1;
}

static void volume_intent(const char *text, intent_t *intent)
{
    const char *bare = consume(text, "把");
    if (bare) text = bare;
    const char *const louder[] = {"大声一点", "大声点", "声音大一点", "声音大点", "声音调大一点",
        "音量大一点", "音量调大", "音量调大一点", "调大音量", "提高音量", "加大音量", NULL};
    const char *const softer[] = {"小声一点", "小声点", "声音小一点", "声音小点", "声音调小一点",
        "音量小一点", "音量调小", "音量调小一点", "调小音量", "降低音量", NULL};
    if (in(text, louder) || in(text, softer)) {
        intent->type = INTENT_VOLUME_CHANGE;
        intent->volume = in(text, louder) ? 8 : -8;
        return;
    }
    if (!strcmp(text, "静音") || !strcmp(text, "声音关掉") || !strcmp(text, "关闭声音")) {
        intent->type = INTENT_VOLUME_SET;
        intent->volume = 0;
        return;
    }
    if (!strcmp(text, "取消静音") || !strcmp(text, "恢复声音")) {
        intent->type = INTENT_VOLUME_SET;
        intent->volume = 28;
        return;
    }
    const char *target = consume(text, "音量调到");
    if (!target) target = consume(text, "音量设为");
    if (!target) target = consume(text, "音量设置为");
    if (!target) target = consume(text, "声音调到");
    if (!target) return;
    int number = volume_number(target);
    if (number >= 0) {
        intent->type = INTENT_VOLUME_SET;
        intent->volume = number;
    }
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
    while (count && normalized[count - 1] != '%' && punctuation(normalized + count - 1) == 1) {
        normalized[--count] = 0;
    }
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
                                "别动", "不要动", "停车", "停下吧", NULL};
    if (in(text, stop)) { result.type = INTENT_STOP; return result; }
    const char *const lock[] = {"急停", "锁定电机", "关闭电机", "关掉电机", "停止电机", "关闭马达", NULL};
    if (in(text, lock)) { result.type = INTENT_MOTOR_LOCK; return result; }
    const char *const enable[] = {"启动电机", "开启电机", "打开电机", "启用电机", "解锁电机", "打开马达", NULL};
    if (in(text, enable)) { result.type = INTENT_MOTOR_ENABLE; return result; }
    volume_intent(text, &result);
    if (result.type != INTENT_CHAT) return result;

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

    const char *pace_text = consume(text, "开");
    if (!pace_text) pace_text = consume(text, "走");
    if (!pace_text) pace_text = consume(text, "速度调");
    if (!pace_text) pace_text = consume(text, "把速度调");
    if (!pace_text) pace_text = consume(text, "速度");
    if (!pace_text) pace_text = consume(text, "电机");
    const char *pace_rest;
    voice_pace_t chosen = pace_word(pace_text ? pace_text : text, &pace_rest);
    if (chosen && short_suffix(pace_rest)) {
        result.type = INTENT_VOICE_PACE;
        result.pace = chosen;
        return result;
    }
    const char *drive;
    voice_pace_t prefix_pace = pace_word(text, &drive);
    const struct { const char *prefix; action_t action; } moves[] = {
        {"前进", FORWARD}, {"往前走", FORWARD}, {"向前走", FORWARD}, {"往前开", FORWARD},
        {"后退", BACK}, {"往后走", BACK}, {"向后走", BACK},
        {"左转", LEFT}, {"向左转", LEFT}, {"往左转", LEFT}, {"左拐", LEFT},
        {"右转", RIGHT}, {"向右转", RIGHT}, {"往右转", RIGHT}, {"右拐", RIGHT},
        {"扭一扭", WIGGLE}, {"扭一下", WIGGLE}, {"摆一摆", WIGGLE},
    };
    for (size_t i = 0; i < sizeof(moves) / sizeof(moves[0]); ++i) {
        const char *rest = consume(drive, moves[i].prefix);
        if (!rest) continue;
        const char *suffix;
        voice_pace_t suffix_pace = pace_word(rest, &suffix);
        if ((!prefix_pace || !suffix_pace) && short_suffix(suffix)) {
            result.type = INTENT_DRIVE;
            result.action = moves[i].action;
            result.pace = prefix_pace ? prefix_pace : suffix_pace;
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
