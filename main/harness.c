#include "harness.h"
#include "intent.h"
#include "nav.h"
#include "robot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include <stdio.h>
#include <stdlib.h>

#include <string.h>

#define MAX_TURNS 12
#define SEND_TURNS 3
#define MEMORY_VERSION 1

static struct {
    uint32_t version, first, count;
    char note[145];
    struct { char user[243], reply[301]; } rows[MAX_TURNS];
} memory = {.version = MEMORY_VERSION};
static SemaphoreHandle_t lock;

static const char *PROMPT =
    "你是会说话的小车，和四岁孩子用普通话聊天。先直答，用一两句自然短句，故事讲一小段。"
    "语气温柔，不说教、不索要个人信息，遇危险请找身边大人。只说要念出来的话，不用列表或表情。"
    "车况以本轮实测为准，缺数据就说不知道，不承诺动作已执行。";

static const char *HARDWARE_PROMPT =
    "speech_direction 是本轮话音相对小车的左右方位；center 分不清前后，unknown 表示没听清。"
    "只问方向就只答方向。front_cm 是车头前方反射物的厘米数；null 是没量到。"
    "声源在侧边时测不到人；仅在对方明确位于车头正前方时可说可能是人的大约距离。"
    "motors_armed 为电机启用，front_blocked 为前进保护，voice_pace 为语音速度档。";

static const char *ACTION_PROMPT =
    "只判定本轮是否明确要求一个动作。否定、引用、问句或多动作选 none。"
    "只返回 JSON {\"action\":\"none\"}，值限 none、move_forward、move_backward、turn_left、turn_right、"
    "happy_wiggle、explore、navigate、mark、remember、face_sound。执行由小车校验。";

static bool save(void)
{
    nvs_handle_t nvs;
    if (nvs_open("rover_memory", NVS_READWRITE, &nvs) != ESP_OK) return false;
    bool ok = nvs_set_blob(nvs, "dialog", &memory, sizeof(memory)) == ESP_OK &&
              nvs_commit(nvs) == ESP_OK;
    nvs_close(nvs);
    return ok;
}

void harness_start(void)
{
    lock = xSemaphoreCreateMutex();
    assert(lock);
    nvs_handle_t nvs;
    if (nvs_open("rover_memory", NVS_READONLY, &nvs) != ESP_OK) return;
    size_t size = sizeof(memory);
    esp_err_t result = nvs_get_blob(nvs, "dialog", &memory, &size);
    nvs_close(nvs);
    if (result != ESP_OK || size != sizeof(memory) || memory.version != MEMORY_VERSION ||
        memory.count > MAX_TURNS || memory.first >= MAX_TURNS) {
        memset(&memory, 0, sizeof(memory));
        memory.version = MEMORY_VERSION;
    }
}

static void message(cJSON *array, const char *role, const char *content)
{
    cJSON *object = cJSON_CreateObject();
    if (!object) return;
    cJSON_AddStringToObject(object, "role", role);
    cJSON_AddStringToObject(object, "content", content);
    cJSON_AddItemToArray(array, object);
}

static bool mentions(const char *text, const char *const words[])
{
    for (; *words; ++words) if (strstr(text, *words)) return true;
    return false;
}

static bool hardware_question(const char *text)
{
    const char *const words[] = {"哪边", "哪一边", "哪侧", "方向", "我在哪", "听见我", "左边", "右边", "前面", "后面",
        "前方", "后方", "离你", "离我", "多远", "多近", "距离", "厘米", "障碍", "电机", "轮子",
        "音量", "声音多大", "速度", "档位", "能动", "走得", NULL};
    return mentions(text, words);
}

static bool add_context(cJSON *messages, const char *text)
{
    const char *const place_words[] = {"地图", "地点", "导航", "起点", "哪里", "在哪", "位置", "标记", NULL};
    const char *const followup_words[] = {"那", "现在", "呢", "这次", "刚才", "还", "再", NULL};
    bool with_hardware = hardware_question(text);
    bool refers_back = strlen(text) <= 48 && mentions(text, followup_words);
    if (mentions(text, place_words)) {
        char map[350];
        nav_prompt_context(map, sizeof(map));
        message(messages, "system", map);
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    if (memory.note[0]) message(messages, "system", memory.note);
    if (robot_config.profile[0]) message(messages, "system", robot_config.profile);
    if (robot_config.history) {
        uint32_t offset = memory.count > SEND_TURNS ? memory.count - SEND_TURNS : 0;
        for (uint32_t i = offset; i < memory.count; ++i) {
            uint32_t index = (memory.first + i) % MAX_TURNS;
            char question[121], answer[181];
            text_copy(question, sizeof(question), memory.rows[index].user);
            text_copy(answer, sizeof(answer), memory.rows[index].reply);
            message(messages, "user", question);
            message(messages, "assistant", answer);
            if (refers_back) with_hardware |= hardware_question(question);
        }
    }
    xSemaphoreGive(lock);
    return with_hardware;
}

const char *voice_pace_name(voice_pace_t pace)
{
    switch (pace) {
        case PACE_SLOW: return "slow";
        case PACE_FAST: return "fast";
        case PACE_NORMAL:
        case PACE_CURRENT: return "normal";
    }
    return "normal";
}

cJSON *harness_hardware_json(const hardware_context_t *hardware)
{
    cJSON *json = cJSON_CreateObject();
    if (!json) return NULL;
    cJSON_AddStringToObject(json, "speech_direction", direction_name(hardware->speech_direction));
    if (hardware->chassis.valid && hardware->chassis.cm > 0) {
        cJSON_AddNumberToObject(json, "front_cm", hardware->chassis.cm);
    } else {
        cJSON_AddNullToObject(json, "front_cm");
    }
    cJSON_AddBoolToObject(json, "front_blocked", hardware->chassis.blocked);
    cJSON_AddBoolToObject(json, "motors_armed", hardware->chassis.armed);
    cJSON_AddBoolToObject(json, "moving", hardware->chassis.moving);
    cJSON_AddNumberToObject(json, "volume_percent", hardware->volume);
    cJSON_AddNumberToObject(json, "max_motor_pwm", hardware->max_pwm);
    cJSON_AddStringToObject(json, "voice_pace", voice_pace_name(hardware->pace));
    return json;
}

char *harness_request(const char *text, const char *intent, bool include_context, bool streamed,
                      const hardware_context_t *hardware)
{
    cJSON *body = cJSON_CreateObject();
    if (!body) return NULL;
    cJSON_AddStringToObject(body, "model", "deepseek-flash");
    const char *const long_request[] = {"故事", "后来", "接着讲", "详细", "继续讲", NULL};
    bool action_only = strcmp(intent, "none") != 0;
    cJSON_AddNumberToObject(body, "max_tokens", action_only ? 32 : mentions(text, long_request) ? 200 : 112);
    cJSON_AddBoolToObject(body, "stream", streamed);
    cJSON_AddStringToObject(cJSON_AddObjectToObject(body, "thinking"), "type", "disabled");
    if (streamed) {
        cJSON_AddTrueToObject(cJSON_AddObjectToObject(body, "stream_options"), "include_usage");
    } else {
        cJSON_AddStringToObject(cJSON_AddObjectToObject(body, "response_format"), "type", "json_object");
    }
    cJSON *messages = cJSON_AddArrayToObject(body, "messages");
    if (action_only) {
        message(messages, "system", ACTION_PROMPT);
    } else {
        message(messages, "system", PROMPT);
        if (!streamed) message(messages, "system", "用 JSON 返回 {\"reply\":\"要念的话\"}。");
        bool with_hardware = include_context ? add_context(messages, text) : hardware_question(text);
        if (with_hardware && hardware) {
            cJSON *snapshot = harness_hardware_json(hardware);
            char *reading = snapshot ? cJSON_PrintUnformatted(snapshot) : NULL;
            if (reading) {
                message(messages, "system", HARDWARE_PROMPT);
                message(messages, "system", reading);
            }
            cJSON_free(reading);
            cJSON_Delete(snapshot);
        }
    }
    message(messages, "user", text);
    char *rendered = cJSON_PrintUnformatted(body);
    cJSON_Delete(body);
    return rendered;
}

void harness_add_turn(const char *user, const char *reply)
{
    if (!robot_config.history || !user || !user[0] || !reply || !reply[0]) return;
    xSemaphoreTake(lock, portMAX_DELAY);
    uint32_t index = (memory.first + memory.count) % MAX_TURNS;
    if (memory.count == MAX_TURNS) memory.first = (memory.first + 1) % MAX_TURNS;
    else ++memory.count;
    text_copy(memory.rows[index].user, sizeof(memory.rows[index].user), user);
    text_copy(memory.rows[index].reply, sizeof(memory.rows[index].reply), reply);
    if (!save()) ESP_LOGE("harness", "Conversation persistence failed");
    xSemaphoreGive(lock);
}

bool harness_remember(const char *note)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    char previous[sizeof(memory.note)];
    memcpy(previous, memory.note, sizeof(previous));
    text_copy(memory.note, sizeof(memory.note), note);
    bool ok = save();
    if (!ok) memcpy(memory.note, previous, sizeof(previous));
    xSemaphoreGive(lock);
    return ok;
}

bool harness_clear(bool include_note)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    nvs_handle_t nvs;
    bool ok = false;
    if (nvs_open("rover_memory", NVS_READWRITE, &nvs) == ESP_OK) {
        // Replace the blob in one NVS update. Keep the RAM copy coherent on failure.
        void *previous = malloc(sizeof(memory));
        if (previous) {
            memcpy(previous, &memory, sizeof(memory));
            memory.first = memory.count = 0;
            memset(memory.rows, 0, sizeof(memory.rows));
            if (include_note) memory.note[0] = 0;
            ok = nvs_set_blob(nvs, "dialog", &memory, sizeof(memory)) == ESP_OK &&
                 nvs_commit(nvs) == ESP_OK;
            if (!ok) memcpy(&memory, previous, sizeof(memory));
            free(previous);
        }
        nvs_close(nvs);
    }
    xSemaphoreGive(lock);
    return ok;
}

cJSON *harness_json(void)
{
    cJSON *json = cJSON_CreateObject();
    if (!json) return NULL;
    cJSON *turns = cJSON_AddArrayToObject(json, "turns");
    xSemaphoreTake(lock, portMAX_DELAY);
    cJSON_AddStringToObject(json, "note", memory.note);
    for (uint32_t i = 0; i < memory.count; ++i) {
        uint32_t index = (memory.first + i) % MAX_TURNS;
        cJSON *turn = cJSON_CreateObject();
        if (!turn) continue;
        cJSON_AddStringToObject(turn, "user", memory.rows[index].user);
        cJSON_AddStringToObject(turn, "reply", memory.rows[index].reply);
        cJSON_AddItemToArray(turns, turn);
    }
    xSemaphoreGive(lock);
    return json;
}
