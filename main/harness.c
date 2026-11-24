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
    "用小动物、玩具、颜色等熟悉的东西解释，语气温柔自然，遇到听不清的地方请她慢慢说。"
    "如果需要追问，一次只问一个简单的问题。害怕或难过时先接住她的感受。"
    "不要说教、吓唬、故意装幼稚，也别把每条回答都变成夸奖。"
    "reply 是会直接念出来的话，使用普通标点，不要 Markdown、列表、表情或技术术语。"
    "不要索要住址、联系方式等个人资料；遇到需要照顾者帮忙的情况，可以说找身边的大人。"
    "历史对话和偏好可以帮助理解话题，年龄和说话方式仍按本条规则。"
    "始终返回 json，格式示例 {\"reply\":\"好的\",\"action\":\"none\"}。"
    "action 只可用 none、stop、move_forward、move_backward、turn_left、turn_right、"
    "happy_wiggle、explore、navigate、mark、remember、face_sound。"
    "仅当当前句明确要求单个动作时使用动作值，历史和偏好只做对话参考。"
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

char *harness_request(const char *text, const char *intent)
{
    cJSON *body = cJSON_CreateObject();
    if (!body) return NULL;
    cJSON_AddStringToObject(body, "model", "deepseek-flash");
    cJSON_AddNumberToObject(body, "max_tokens", 230);
    cJSON_AddFalseToObject(body, "stream");
    cJSON_AddStringToObject(cJSON_AddObjectToObject(body, "thinking"), "type", "disabled");
    cJSON_AddStringToObject(cJSON_AddObjectToObject(body, "response_format"), "type", "json_object");
    cJSON *messages = cJSON_AddArrayToObject(body, "messages");
    message(messages, "system", PROMPT);

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
