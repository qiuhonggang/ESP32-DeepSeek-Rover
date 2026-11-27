#pragma once
#include "robot.h"

typedef enum {
    INTENT_CHAT, INTENT_STOP, INTENT_DRIVE, INTENT_EXPLORE, INTENT_GOTO, INTENT_MARK,
    INTENT_REMEMBER, INTENT_FACE_SOUND, INTENT_APPROACH,
    INTENT_VOLUME_SET, INTENT_VOLUME_CHANGE, INTENT_VOICE_PACE,
    INTENT_MOTOR_LOCK, INTENT_MOTOR_ENABLE, INTENT_END_DIALOG,
} intent_type_t;

typedef enum { PACE_CURRENT, PACE_SLOW, PACE_NORMAL, PACE_FAST } voice_pace_t;

typedef struct {
    intent_type_t type;
    action_t action;
    voice_pace_t pace;
    int volume;
    char argument[145];
} intent_t;

void text_copy(char *target, size_t length, const char *source);
intent_t intent_from_transcript(const char *transcript, const wake_option_t *wake_word);
bool intent_model_candidate(const char *transcript, intent_t parsed);
intent_t intent_from_model(const char *transcript, intent_t parsed, const char *action, const char *pace);
direction_t intent_approach_direction(const char *transcript, direction_t direction);
