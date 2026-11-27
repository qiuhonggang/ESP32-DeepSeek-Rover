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

static bool drive_suffix(const char *suffix)
{
    static const char *const amounts[] = {
        "一小步", "一点儿", "一点点", "一下子", "一点", "一下", "一步", NULL,
    };
    static const char *const endings[] = {"", "吧", "呀", "哦", "好吗", "好不好", "可以吗", "行吗", "吗", NULL};
    for (size_t i = 0; amounts[i]; ++i) {
        const char *after = consume(suffix, amounts[i]);
        if (after) return in(after, endings);
    }
    return in(suffix, endings);
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

static void drive_intent(const char *text, intent_t *intent)
{
    static const char *const lead_ins[] = {
        "能不能", "可不可以", "可以", "能", "麻烦你", "帮我", "给我", "请", "再", "先", NULL,
    };
    for (int i = 0; i < 3; ++i) {
        const char *after = NULL;
        for (size_t j = 0; lead_ins[j] && !after; ++j) after = consume(text, lead_ins[j]);
        if (!after) break;
        text = after;
        size_t skip;
        while ((skip = pause_separator(text)) != 0) text += skip;
    }
    const char *drive;
    voice_pace_t prefix_pace = pace_word(text, &drive);
    static const struct { const char *phrase; action_t action; } moves[] = {
        {"前进", FORWARD}, {"往前", FORWARD}, {"向前", FORWARD}, {"朝前", FORWARD},
        {"往前走", FORWARD}, {"向前走", FORWARD}, {"往前开", FORWARD}, {"向前开", FORWARD},
        {"往前挪", FORWARD}, {"向前挪", FORWARD}, {"朝前走", FORWARD}, {"朝前开", FORWARD},
        {"后退", BACK}, {"退后", BACK}, {"往后", BACK}, {"向后", BACK}, {"倒车", BACK},
        {"往后走", BACK}, {"向后走", BACK}, {"往后退", BACK}, {"向后退", BACK},
        {"往后倒", BACK}, {"往后挪", BACK}, {"退", BACK},
        {"左转", LEFT}, {"向左转", LEFT}, {"往左转", LEFT}, {"左拐", LEFT},
        {"朝左转", LEFT}, {"朝左边转", LEFT}, {"往左边转", LEFT},
        {"右转", RIGHT}, {"向右转", RIGHT}, {"往右转", RIGHT}, {"右拐", RIGHT},
        {"朝右转", RIGHT}, {"朝右边转", RIGHT}, {"往右边转", RIGHT},
        {"扭一扭", WIGGLE}, {"扭一下", WIGGLE}, {"摆一摆", WIGGLE},
    };
    for (size_t i = 0; i < sizeof(moves) / sizeof(moves[0]); ++i) {
        const char *rest = consume(drive, moves[i].phrase);
        if (!rest) continue;
        const char *suffix;
        voice_pace_t suffix_pace = pace_word(rest, &suffix);
        if ((!prefix_pace || !suffix_pace) && drive_suffix(suffix)) {
            intent->type = INTENT_DRIVE;
            intent->action = moves[i].action;
            intent->pace = prefix_pace ? prefix_pace : suffix_pace;
            return;
        }
    }
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
    const char *const lead_ins[] = {"等一下", "等等", "我说", "帮我", "给我"};
    for (size_t i = 0; i < sizeof(lead_ins) / sizeof(lead_ins[0]); ++i) {
        const char *after = consume(text, lead_ins[i]);
        if (after) {
            text = after;
            size_t skip;
            while ((skip = pause_separator(text)) != 0) text += skip;
        }
    }
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
    if (*transcript) return result;
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

    const char *const end_dialog[] = {"不聊了", "先不聊了", "退出对话", "结束对话", NULL};
    if (in(text, end_dialog)) { result.type = INTENT_END_DIALOG; return result; }
    const char *const stop[] = {"停", "停下", "停下来", "停止", "停止巡游", "停止导航",
                                "别动", "不要动", "停车", "停下吧", "停一下", "先停一下", "快停下", NULL};
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
    drive_intent(text, &result);
    if (result.type == INTENT_DRIVE) return result;

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

static bool contains_any(const char *text, const char *const *words)
{
    for (size_t i = 0; words[i]; ++i) {
        if (strstr(text, words[i])) return true;
    }
    return false;
}

static const char *const approach_words[] = {
    "过来", "过來", "过莱", "到我", "来我", "来这", "到这", "靠近", "靠过来", "向我开", "朝我开", NULL,
};

bool intent_model_candidate(const char *transcript, intent_t parsed)
{
    if (parsed.type != INTENT_CHAT) return true;
    static const char *const cues[] = {
        "前", "后", "左", "右", "转", "停车", "停下", "停住", "不动", "别动", "别走", "别开",
        "走", "倒", "退", "开车", "挪", "这边", "绕", "扭", "摆", "网钱", "往钱", "往厚",
        "动一", "巡游", "建图", "面向我", NULL,
    };
    return contains_any(transcript, cues) || contains_any(transcript, approach_words);
}

intent_t intent_from_model(const char *transcript, intent_t parsed, const char *action, const char *pace)
{
    const intent_t no_action = {.type = INTENT_CHAT, .action = STOP};
    if (!action || !intent_model_candidate(transcript, parsed)) return no_action;
    static const char *const reported_or_composite[] = {
        "说过", "他说", "她说", "我说过", "我没说", "说一遍", "命令是", "比如", "例如",
        "然后", "接着再", "之后再", "同时", "还是", "先前进再", "先往前再", "先右转再", "先左转再",
        "怎么", "为什么", "什么", "是不是", "会不会", "多远", "多快", "有没有",
        "“", "”", "‘", "’", "\"", NULL,
    };
    if (contains_any(transcript, reported_or_composite)) return no_action;
    if (!strcmp(action, "stop")) return (intent_t){.type = INTENT_STOP};
    static const char *const negative[] = {
        "不要", "别", "不用", "不许", "没让", "并非", "不准", NULL,
    };
    if (contains_any(transcript, negative)) return no_action;
    const char *unable = transcript;
    while ((unable = strstr(unable, "不能")) != NULL) {
        if (unable - transcript < 3 || memcmp(unable - 3, "能", 3)) return no_action;
        unable += strlen("不能");
    }

    static const struct { const char *name; action_t motion; } motions[] = {
        {"move_forward", FORWARD}, {"move_backward", BACK}, {"turn_left", LEFT},
        {"turn_right", RIGHT}, {"happy_wiggle", WIGGLE},
    };
    for (size_t i = 0; i < sizeof(motions) / sizeof(motions[0]); ++i) {
        if (strcmp(action, motions[i].name)) continue;
        if (parsed.type != INTENT_CHAT && (parsed.type != INTENT_DRIVE || parsed.action != motions[i].motion)) {
            return no_action;
        }
        voice_pace_t speed = parsed.pace;
        if (!speed && pace) {
            if (!strcmp(pace, "slow")) speed = PACE_SLOW;
            if (!strcmp(pace, "fast")) speed = PACE_FAST;
            if (!strcmp(pace, "normal")) speed = PACE_NORMAL;
        }
        return (intent_t){.type = INTENT_DRIVE, .action = motions[i].motion, .pace = speed};
    }
    if (!strcmp(action, "approach") && (parsed.type == INTENT_CHAT || parsed.type == INTENT_GOTO) &&
        contains_any(transcript, approach_words)) {
        return (intent_t){.type = INTENT_APPROACH};
    }
    if (!strcmp(action, "face_sound") && (parsed.type == INTENT_CHAT || parsed.type == INTENT_FACE_SOUND)) {
        return (intent_t){.type = INTENT_FACE_SOUND};
    }
    if ((!strcmp(action, "explore") && parsed.type == INTENT_EXPLORE) ||
        (!strcmp(action, "navigate") && parsed.type == INTENT_GOTO) ||
        (!strcmp(action, "mark") && parsed.type == INTENT_MARK) ||
        (!strcmp(action, "remember") && parsed.type == INTENT_REMEMBER)) return parsed;
    return no_action;
}

direction_t intent_approach_direction(const char *transcript, direction_t direction)
{
    static const char *const in_front[] = {
        "我在你前面", "我在你正前方", "我在你前方", "我在车头前面", "我在车头前方",
        "我在车头的前面", "我就在你前面", NULL,
    };
    if (direction == DIR_CENTER && !contains_any(transcript, in_front)) return DIR_UNKNOWN;
    return direction;
}
