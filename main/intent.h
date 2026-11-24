#pragma once
#include "robot.h"

typedef enum {
    INTENT_CHAT, INTENT_STOP, INTENT_DRIVE, INTENT_EXPLORE, INTENT_GOTO, INTENT_MARK,
    INTENT_REMEMBER, INTENT_FACE_SOUND,
} intent_type_t;

typedef struct {
    intent_type_t type;
    action_t action;
    char argument[145];
} intent_t;

void text_copy(char *target, size_t length, const char *source);
intent_t intent_from_transcript(const char *transcript, const wake_option_t *wake_word);
