#pragma once
#include <stdbool.h>
#include "cJSON.h"

cJSON *cloud_json(void);
bool cloud_debug_text(const char *text, const char *command);
