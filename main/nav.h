#pragma once
#include "cJSON.h"
#include "grid.h"
#include "robot.h"

void nav_start(void);
void nav_stop(bool disarm);
bool nav_set_origin(bool clear, heading_t heading);
bool nav_explore(void);
bool nav_explore_for_turn(uint32_t turn);
// DIR_CENTER is accepted only when the caller has confirmed the speaker is in front.
bool nav_approach_for_turn(direction_t direction, int speed, uint32_t turn);
bool nav_go(cell_pos_t goal);
bool nav_go_name(const char *name);
bool nav_go_name_for_turn(const char *name, uint32_t turn);
bool nav_mark(const char *name);
bool nav_mark_for_turn(const char *name, uint32_t turn);
bool nav_save(void);
bool nav_calibrate(action_t action);
uint32_t nav_manual(action_t action, int speed, int duration_ms, motion_source_t source, int wheel);
uint32_t nav_cloud_motion(action_t action, int speed, int duration_ms, uint32_t turn);
cJSON *nav_json(void);
void nav_prompt_context(char *buffer, size_t length);
