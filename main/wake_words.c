#include "robot.h"

const wake_option_t wake_options[] = {
    // Indices are saved in NVS. Append new entries and keep all existing indices stable.
    {"wn9_hai1xiao3xiang4_tts3", "嗨小象", {"hi小象"}},
    {"wn9_himiaomiao_tts", "嗨喵喵", {"hi喵喵"}},
    {"wn9_nihaoxiaozhi_tts", "你好小智", {"你好小志"}},
    {"wn9_hilexin", "嗨，乐鑫", {"hi乐鑫", "嗨乐欣"}},
    {"wn9_xiaoyaxiaoya_tts2", "小鸭小鸭", {NULL}},
    {"wn9_xiaoluxiaolu_tts2", "小鹿小鹿", {"小路小路", "小露小露"}},
    {"wn9_xiaolongxiaolong_tts", "小龙小龙", {NULL}},
    {"wn9_xiaobinxiaobin_tts", "小冰小冰", {"小滨小滨", "小兵小兵"}},
    {"wn9_xiao3jia1xiao3jia1_tts3", "小珈小珈", {"小嘉小嘉", "小佳小佳", "小家小家"}},
    {"wn9_hai1xiao3ou1_tts3", "嗨小欧", {"hi小欧", "嗨小鸥"}},
    {"wn9_xiaosurou_tts2", "小酥肉", {NULL}},
    {"wn9l_ni3hao3xing1bao3_tts3", "你好星宝", {"你好星宝儿"}},
    {"wn9_nihaodongdong_tts2", "你好东东", {"你好冬冬", "你好咚咚"}},
    {"wn9_nihaoxiaoan_tts2", "你好小安", {NULL}},
    {"wn9_miaomiaotongxue_tts", "喵喵同学", {"苗苗同学"}},
    {"wn9_xiaomeitongxue_tts", "小美同学", {NULL}},
    {"wn10_shuang1er2", "双儿", {NULL}},
    {"wn10_tu2tu2ni3hao3", "涂涂你好", {"图图你好", "兔兔你好"}},
    {"wn10_dou4bao1dou4bao1", "豆包豆包", {NULL}},
    {"wn10_deepseek", "DeepSeek", {NULL}},
};

const unsigned wake_option_count = sizeof(wake_options) / sizeof(wake_options[0]);
