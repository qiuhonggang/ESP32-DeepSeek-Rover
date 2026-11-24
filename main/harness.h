#pragma once
#include "cJSON.h"
#include "intent.h"

typedef struct {
    direction_t speech_direction;
    motion_status_t chassis;
    int volume, max_pwm;
    voice_pace_t pace;
} hardware_context_t;

void harness_start(void);
const char *voice_pace_name(voice_pace_t pace);
cJSON *harness_hardware_json(const hardware_context_t *hardware);
char *harness_request(const char *text, const char *intent, bool include_context, bool streamed,
                      const hardware_context_t *hardware);
void harness_add_turn(const char *user, const char *reply);
bool harness_remember(const char *note);
bool harness_clear(bool include_note);
cJSON *harness_json(void);
