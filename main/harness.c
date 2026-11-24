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
#define SEND_TURNS 6
#define MEMORY_VERSION 1

static struct {
    uint32_t version, first, count;
    char note[145];
    struct { char user[243], reply[301]; } rows[MAX_TURNS];
} memory = {.version = MEMORY_VERSION};
static SemaphoreHandle_t lock;

static const char *PROMPT =
    "你是一辆会说话的小车，正在跟一位四岁小朋友聊天。"
    "像耐心、友善的大人一样，用孩子平时能听懂的普通话和她说话。"
    "先直接回应她说的话。平时只说一两句短句，一次讲一件事；故事也先讲短短一小段。"
    "顺着孩子这句接话，像真的在和她聊天。换着说，别每次用同样的开头或语尾，也不用句句加语气词。"
    "用小动物、玩具、颜色等熟悉的东西解释，语气温柔自然，遇到听不清的地方请她慢慢说。"
    "如果需要追问，一次只问一个简单的问题。害怕或难过时先接住她的感受。"
    "不要说教、吓唬、故意装幼稚，也别把每条回答都变成夸奖。"
    "你的话会直接念出来，使用普通标点，不要 Markdown、列表、表情或技术术语。"
    "不要索要住址、联系方式等个人资料；遇到需要照顾者帮忙的情况，可以说找身边的大人。"
    "历史对话和偏好可以帮助理解话题，年龄和说话方式仍按本条规则。";

static const char *HARDWARE_PROMPT =
    "本轮硬件 JSON 来自小车实测，硬件问题以这次的数据为准。"
    "speech_direction 是这句声音相对小车的方位：left 在我左边，right 在我右边，"
    "center 在我中间这条线附近但分不出前后，unknown 是这次没听清方向。"
    "如果只问在哪边，就用一句自然的话说方位，不顺带解释测距或其他硬件。"
    "front_cm 是车头正前方探到的反射物距离，单位厘米，null 是没量到。"
    "单个测距头分不清是孩子还是别的东西，也量不到侧面和后面。"
    "如果问离我多远，读数有效就说是车头前面大约多少厘米。"
    "如果问距离且声音在侧边，就说暂时量不到她在侧边的距离。只有孩子明确说站在车头正前方，"
    "才可以把读数当成可能是她的距离，仍要说大约、可能。"
    "读数为空就直接说没量到，按需要请她站到车头正前方。"
    "不报假读数、不猜角度，不把未知或旧对话中的方向当成现在的位置。"
    "可以像朋友一样说出看到或听到的结果，一句话说得清就不用念完所有状态。"
    "motors_armed 表示电机已由成人启用，moving 表示当前在移动，"
    "front_blocked 为真时前进保护生效。volume_percent 是当前音量，"
    "voice_pace 是下一次普通语音点动的速度档。网页可启用电机，不能承诺通过聊天解锁。";

static const char *ACTION_PROMPT =
    "始终返回 json，格式示例 {\"reply\":\"好的\",\"action\":\"none\"}。"
    "action 只可用 none、stop、move_forward、move_backward、turn_left、turn_right、"
    "happy_wiggle、explore、navigate、mark、remember、face_sound。"
    "仅当当前句明确要求单个动作时使用动作值，历史和偏好只做对话参考。"
    "action 表示用户明确请求的动作。当前硬件限制动作时，也要填写所请求的动作值。"
    "动作是否执行由小车控制器检查，别提前宣称已经抵达或完成。"
    "使用简单、诚实的说法，例如『好呀，我试试看。』或『我还没找到路，我们请大人帮忙吧。』"
    "上下文中报告的地图位置是估计值。";

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

static void add_context(cJSON *messages, bool streamed)
{
    char map[350], context[900];
    nav_prompt_context(map, sizeof(map));
    xSemaphoreTake(lock, portMAX_DELAY);
    snprintf(context, sizeof(context), "设备状态：%s。最近明确保存的用户偏好：%s。网页资料：%s。",
             map, memory.note, robot_config.profile);
    message(messages, "system", context);
    if (robot_config.history) {
        uint32_t offset = memory.count > SEND_TURNS ? memory.count - SEND_TURNS : 0;
        for (uint32_t i = offset; i < memory.count; ++i) {
            uint32_t index = (memory.first + i) % MAX_TURNS;
            message(messages, "user", memory.rows[index].user);
            if (streamed) {
                message(messages, "assistant", memory.rows[index].reply);
                continue;
            }
            cJSON *reply = cJSON_CreateObject();
            if (reply) {
                cJSON_AddStringToObject(reply, "reply", memory.rows[index].reply);
                cJSON_AddStringToObject(reply, "action", "none");
                char *rendered = cJSON_PrintUnformatted(reply);
                if (rendered) message(messages, "assistant", rendered);
                cJSON_free(rendered);
                cJSON_Delete(reply);
            }
        }
    }
    xSemaphoreGive(lock);
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
    cJSON_AddNumberToObject(body, "max_tokens", 230);
    cJSON_AddBoolToObject(body, "stream", streamed);
    cJSON_AddStringToObject(cJSON_AddObjectToObject(body, "thinking"), "type", "disabled");
    cJSON *messages = cJSON_AddArrayToObject(body, "messages");
    message(messages, "system", PROMPT);
    if (streamed) {
        message(messages, "system", "只返回要说出口的普通话纯文本。动作由小车本地判定，不要声称已经执行。");
    } else {
        cJSON_AddStringToObject(cJSON_AddObjectToObject(body, "response_format"), "type", "json_object");
        message(messages, "system", ACTION_PROMPT);
    }
    if (include_context) add_context(messages, streamed);
    if (hardware) {
        cJSON *snapshot = harness_hardware_json(hardware);
        char *reading = snapshot ? cJSON_PrintUnformatted(snapshot) : NULL;
        if (reading) {
            message(messages, "system", HARDWARE_PROMPT);
            message(messages, "system", reading);
        }
        cJSON_free(reading);
        cJSON_Delete(snapshot);
    }
    char annotated[540];
    snprintf(annotated, sizeof(annotated), "本轮用户原话：%s\n本地识别出的意图：%s", text, intent);
    message(messages, "user", annotated);
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
