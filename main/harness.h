#pragma once
#include "cJSON.h"
#include <stdbool.h>

void harness_start(void);
char *harness_request(const char *text, const char *intent);
void harness_add_turn(const char *user, const char *reply);
bool harness_remember(const char *note);
bool harness_clear(bool include_note);
cJSON *harness_json(void);
