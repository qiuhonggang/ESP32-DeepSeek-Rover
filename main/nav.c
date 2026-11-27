#include "nav.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#define POINTS 6
#define LABEL_BYTES 37
#define MAP_VERSION 1

typedef enum { NAV_IDLE, NAV_EXPLORE, NAV_GOAL, NAV_APPROACH } nav_mode_t;
typedef enum { DECIDE, ORIENT, MEASURE } nav_phase_t;

static const cell_pos_t HOME = {GRID_CENTER, GRID_CENTER};
static struct {
    uint32_t version;
    int cell_cm;
    grid_t grid;
    struct { char name[LABEL_BYTES]; int16_t x, y; } points[POINTS];
} saved_map;
static SemaphoreHandle_t lock;
static bool map_task_ready;
static struct {
    cell_pos_t pos, goal, next, target;
    heading_t heading, desired;
    nav_mode_t mode;
    nav_phase_t phase;
    action_t issued;
    uint32_t ticket, measure_seq;
    uint32_t approach_turn;
    int approach_speed;
    int readings[3], count;
    bool pose, scanning;
    int steps;
    int64_t settle, deadline, run_until;
    const char *message;
} nav = {.message = "origin_required"};

static const char *const MODE_NAMES[] = {"idle", "explore", "route", "approach"};

static void end(const char *message, bool disarm)
{
    bool approaching = nav.mode == NAV_APPROACH;
    if (nav.ticket && !approaching) {
        motion_status_t status = motion_status();
        if (nav.ticket == status.ticket && status.result == MOTION_DONE) {
            if (nav.issued == FORWARD) nav.pos = nav.next;
            if (nav.issued == LEFT) nav.heading = (nav.heading + 3) % 4;
            if (nav.issued == RIGHT) nav.heading = (nav.heading + 1) % 4;
        } else {
            nav.pose = false;
        }
    }
    nav.ticket = 0;
    nav.mode = NAV_IDLE;
    nav.phase = DECIDE;
    nav.message = approaching || nav.pose ? message : "origin_required";
    motion_stop(disarm);
}

static bool writable_name(const char *name)
{
    if (!name) return false;
    size_t bytes = strlen(name);
    if (bytes == 0 || bytes >= LABEL_BYTES || !strcmp(name, "起点")) return false;
    for (size_t i = 0; i < bytes; ++i) {
        unsigned char c = name[i];
        if (c < 32 || c == 127) return false;
    }
    return true;
}


bool nav_set_origin(bool clear, heading_t heading)
{
    if (heading < NORTH || heading > WEST) return false;
    xSemaphoreTake(lock, portMAX_DELAY);
    motion_status_t status = motion_status();
    bool ok = nav.mode == NAV_IDLE && !status.moving && status.result != MOTION_RUNNING;
    if (ok) {
        if (clear) {
            memset(&saved_map, 0, sizeof(saved_map));
            saved_map.version = MAP_VERSION;
            saved_map.cell_cm = robot_config.cell_cm;
        }
        nav.pos = HOME;
        nav.goal = HOME;
        nav.target = HOME;
        nav.heading = heading;
        nav.pose = true;
        nav.message = "origin_set";
        grid_set(&saved_map.grid, HOME, CELL_FREE);
    }
    xSemaphoreGive(lock);
    return ok;
}

static bool turn_valid(uint32_t turn)
{
    return turn == 0 || (network_ready() && audio_turn_is_current(turn));
}

static bool begin(nav_mode_t mode, cell_pos_t goal, uint32_t turn)
{
    if (!turn_valid(turn)) return false;
    if (!map_task_ready || !nav.pose || !motion_status().armed || nav.mode != NAV_IDLE ||
        motion_status().result == MOTION_RUNNING || motion_status().moving) return false;
    if (mode == NAV_GOAL && grid_get(&saved_map.grid, goal) != CELL_FREE) return false;
    nav.mode = mode;
    nav.goal = goal;
    nav.target = nav.pos;
    nav.phase = DECIDE;
    nav.ticket = 0;
    nav.steps = 0;
    nav.run_until = esp_timer_get_time() + 135000000;
    nav.message = mode == NAV_EXPLORE ? "exploring" : "planning";
    return true;
}

bool nav_explore_for_turn(uint32_t turn)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    bool ok = begin(NAV_EXPLORE, HOME, turn);
    xSemaphoreGive(lock);
    return ok;
}

bool nav_explore(void) { return nav_explore_for_turn(0); }

bool nav_approach_for_turn(direction_t direction, int speed, uint32_t turn)
{
    if (!turn || direction < DIR_LEFT || direction > DIR_RIGHT) return false;

    xSemaphoreTake(lock, portMAX_DELAY);
    uint32_t ticket = 0;
    if (map_task_ready && nav.mode == NAV_IDLE && turn_valid(turn)) {
        action_t action = direction == DIR_LEFT ? LEFT : direction == DIR_RIGHT ? RIGHT : FORWARD;
        int duration_ms = APPROACH_LIMIT_MS;
        int pwm = speed;
        if (action != FORWARD) {
            // The left and right calibration values describe 90° at nav_speed.
            duration_ms = (action == LEFT ? robot_config.turn_left_ms : robot_config.turn_right_ms) / 2;
            pwm = robot_config.nav_speed;
        }
        int64_t now = esp_timer_get_time();
        ticket = motion_submit(action, pwm, duration_ms, SRC_APPROACH, -1);
        if (ticket) {
            nav.mode = NAV_APPROACH;
            nav.ticket = ticket;
            nav.pose = false;
            nav.issued = action;
            nav.approach_turn = turn;
            nav.approach_speed = speed;
            nav.run_until = now + (int64_t)APPROACH_LIMIT_MS * 1000;
            nav.message = action == FORWARD ? "approach_moving" : "approach_turning";
        }
    }
    xSemaphoreGive(lock);
    return ticket != 0;
}

bool nav_go(cell_pos_t goal)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    bool ok = begin(NAV_GOAL, goal, 0);
    xSemaphoreGive(lock);
    return ok;
}

bool nav_go_name_for_turn(const char *name, uint32_t turn)
{
    if (!name) return false;
    xSemaphoreTake(lock, portMAX_DELAY);
    if (!strcmp(name, "起点")) {
        bool ok = begin(NAV_GOAL, HOME, turn);
        xSemaphoreGive(lock);
        return ok;
    }
    bool ok = false;
    for (int i = 0; i < POINTS; ++i) {
        if (saved_map.points[i].name[0] && !strcmp(name, saved_map.points[i].name)) {
            cell_pos_t goal = {saved_map.points[i].x, saved_map.points[i].y};
            ok = begin(NAV_GOAL, goal, turn);
            break;
        }
    }
    xSemaphoreGive(lock);
    return ok;
}

bool nav_go_name(const char *name) { return nav_go_name_for_turn(name, 0); }

bool nav_mark_for_turn(const char *name, uint32_t turn)
{
    if (!writable_name(name)) return false;
    xSemaphoreTake(lock, portMAX_DELAY);
    bool ok = false;
    if (nav.mode == NAV_IDLE && nav.pose && !motion_status().moving && turn_valid(turn)) {
        int slot = -1;
        for (int i = 0; i < POINTS; ++i) {
            if (!strcmp(name, saved_map.points[i].name)) {
                slot = i;
                break;
            }
            if (slot < 0 && !saved_map.points[i].name[0]) slot = i;
        }
        if (slot >= 0) {
            strlcpy(saved_map.points[slot].name, name, sizeof(saved_map.points[slot].name));
            saved_map.points[slot].x = nav.pos.x;
            saved_map.points[slot].y = nav.pos.y;
            ok = true;
        }
    }
    xSemaphoreGive(lock);
    return ok;
}

bool nav_mark(const char *name) { return nav_mark_for_turn(name, 0); }

void nav_stop(bool disarm)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    const char *message = nav.mode == NAV_APPROACH ? (disarm ? "locked" : "approach_cancelled") : "stopped";
    end(message, disarm);
    xSemaphoreGive(lock);
}

static uint32_t manual(action_t action, int speed, int ms, motion_source_t source, int wheel, uint32_t turn)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    uint32_t ticket = 0;
    if (nav.mode == NAV_IDLE && turn_valid(turn)) {
        ticket = motion_submit(action, speed, ms, source, wheel);
        if (ticket) {
            nav.pose = false;
            nav.message = "origin_required";
        }
    }
    xSemaphoreGive(lock);
    return ticket;
}

uint32_t nav_manual(action_t action, int speed, int duration_ms, motion_source_t source, int wheel)
{
    return manual(action, speed, duration_ms, source, wheel, 0);
}

uint32_t nav_cloud_motion(action_t action, int speed, int duration_ms, uint32_t turn)
{
    return turn ? manual(action, speed, duration_ms, SRC_CLOUD, -1, turn) : 0;
}

bool nav_calibrate(action_t action)
{
    int ms;
    switch (action) {
        case FORWARD: ms = robot_config.step_ms; break;
        case LEFT: ms = robot_config.turn_left_ms; break;
        case RIGHT: ms = robot_config.turn_right_ms; break;
        default: return false;
    }
    return manual(action, robot_config.nav_speed, ms, SRC_NAV, -1, 0) != 0;
}

static void start_measure(int64_t now)
{
    nav.measure_seq = motion_status().measurement;
    nav.count = 0;
    nav.settle = now + 160000;
    nav.deadline = now + 1300000;
    nav.phase = MEASURE;
    nav.message = "measuring";
}

static int scan_sample(motion_status_t status)
{
    if (status.measurement == nav.measure_seq) return -2;
    nav.measure_seq = status.measurement;
    nav.readings[nav.count++] = status.valid ? status.cm : -1;
    if (nav.count < 3) return -2;
    int low = 10000, high = 0, sum = 0, count = 0;
    for (int i = 0; i < 3; ++i) {
        int cm = nav.readings[i];
        if (cm < 0) continue;
        if (cm < low) low = cm;
        if (cm > high) high = cm;
        sum += cm;
        ++count;
    }
    if (count < 2) return -1;
    return count == 3 ? sum - low - high : low;
}

static void choose_next(void)
{
    nav.scanning = false;
    if (nav.mode == NAV_EXPLORE && grid_scans(&saved_map.grid, nav.pos) != 15) {
        uint8_t scans = grid_scans(&saved_map.grid, nav.pos);
        for (int i = 0; i < 4; ++i) {
            heading_t candidate = (nav.heading + i) % 4;
            if (!(scans & (1u << candidate))) {
                nav.desired = candidate;
                nav.scanning = true;
                break;
            }
        }
        nav.target = nav.pos;
    } else {
        cell_pos_t *goal = nav.mode == NAV_GOAL ? &nav.goal : NULL;
        if (!grid_next_step(&saved_map.grid, nav.pos, goal, &nav.next, &nav.target)) {
            end(nav.mode == NAV_GOAL ? "no_route" : "exploration_done", false);
            return;
        }
        if (grid_equal(nav.pos, nav.next)) {
            end("arrived", false);
            return;
        }
        nav.desired = grid_bearing(nav.pos, nav.next);
    }
    nav.phase = ORIENT;
    nav.message = "orienting";
}

static const char *approach_abort_reason(const char *reason)
{
    if (!reason) return "approach_refused";
    if (!strcmp(reason, "obstacle")) return "approach_obstacle";
    if (!strcmp(reason, "echo") || !strcmp(reason, "echo_range") || !strcmp(reason, "stale")) {
        return "approach_no_echo";
    }
    if (!strcmp(reason, "lock")) return "locked";
    if (!strcmp(reason, "stop")) return "approach_cancelled";
    return "approach_refused";
}

static void approach_tick(int64_t now, motion_status_t chassis)
{
    if (!chassis.armed) {
        end("locked", false);
        return;
    }
    if (!turn_valid(nav.approach_turn)) {
        end("approach_cancelled", false);
        return;
    }
    if (now >= nav.run_until) {
        end("approach_timeout", false);
        return;
    }
    if (nav.ticket) {
        if (chassis.ticket != nav.ticket) {
            end("approach_refused", false);
        } else if (chassis.result == MOTION_ABORTED) {
            end(approach_abort_reason(chassis.stop_reason), false);
        } else if (chassis.result == MOTION_DONE) {
            if (nav.issued == FORWARD) {
                end("approach_timeout", false);
            } else {
                nav.ticket = 0;
                nav.measure_seq = chassis.measurement;
                nav.settle = now + 130000;
                nav.message = "approach_measuring";
            }
        }
        return;
    }
    if (!chassis.valid) {
        end("approach_no_echo", false);
        return;
    }
    if (chassis.blocked || chassis.cm <= motion_approach_clearance_cm(FORWARD)) {
        end("approach_obstacle", false);
        return;
    }
    if (now < nav.settle || chassis.measurement == nav.measure_seq ||
        chassis.sample_age_ms < 0 || chassis.sample_age_ms > (now - nav.settle) / 1000) return;

    int remaining_ms = (nav.run_until - now) / 1000;
    if (remaining_ms < 80) {
        end("approach_timeout", false);
        return;
    }
    nav.ticket = motion_submit(FORWARD, nav.approach_speed, remaining_ms, SRC_APPROACH, -1);
    if (nav.ticket) {
        nav.issued = FORWARD;
        nav.message = "approach_moving";
    } else {
        end("approach_refused", false);
    }
}

static void planner_tick(int64_t now)
{
    if (nav.mode == NAV_IDLE) return;
    motion_status_t chassis = motion_status();
    if (nav.mode == NAV_APPROACH) {
        approach_tick(now, chassis);
        return;
    }
    if (now > nav.run_until) {
        end("navigation_timeout", false);
        return;
    }
    if (!chassis.armed || nav.steps >= 32) {
        end(chassis.armed ? "step_limit" : "locked", false);
        return;
    }
    if (nav.ticket) {
        if (chassis.ticket == nav.ticket && chassis.result == MOTION_RUNNING) {
            if (now > nav.deadline) end("motion_timeout", false);
            return;
        }
        if (chassis.ticket != nav.ticket || chassis.result != MOTION_DONE) {
            end("motion_aborted", false);
            return;
        }
        nav.ticket = 0;
        if (nav.issued == FORWARD) {
            nav.pos = nav.next;
            grid_set(&saved_map.grid, nav.pos, CELL_FREE);
            ++nav.steps;
            nav.phase = DECIDE;
        } else {
            nav.heading = (nav.heading + (nav.issued == RIGHT ? 1 : 3)) % 4;
        }
        nav.settle = now + 150000;
        return;
    }
    if (now < nav.settle) return;
    if (nav.phase == DECIDE) {
        choose_next();
        if (nav.mode != NAV_IDLE) nav.deadline = now + 4200000;
        return;
    }
    if (nav.phase == ORIENT) {
        if (nav.desired == nav.heading) {
            start_measure(now);
            return;
        }
        int delta = (nav.desired + 4 - nav.heading) % 4;
        action_t action = delta == 3 ? LEFT : RIGHT;
        int ms = action == LEFT ? robot_config.turn_left_ms : robot_config.turn_right_ms;
        nav.ticket = motion_submit(action, robot_config.nav_speed, ms, SRC_NAV, -1);
        if (nav.ticket) {
            nav.issued = action;
            nav.deadline = now + (int64_t)(ms + 750) * 1000;
        } else if (now > nav.deadline) {
            end("turn_blocked", false);
        }
        return;
    }

    int cm = scan_sample(chassis);
    if (cm == -2 && now < nav.deadline) return;
    int clearance = robot_config.stop_cm + 6;
    if (cm > 0) grid_observe(&saved_map.grid, nav.pos, nav.heading, cm, robot_config.cell_cm, clearance);
    if (nav.scanning) {
        grid_mark_scanned(&saved_map.grid, nav.pos, nav.heading);
        nav.phase = DECIDE;
        return;
    }
    if (cm < robot_config.cell_cm + clearance || !chassis.valid || chassis.blocked ||
        chassis.cm < robot_config.cell_cm + clearance) {
        if (cm <= 0 || !chassis.valid) grid_set(&saved_map.grid, nav.next, CELL_UNKNOWN);
        else grid_set(&saved_map.grid, nav.next, CELL_BLOCKED);
        nav.phase = DECIDE; // Plan again with the updated grid, still at the same cell.
        nav.message = "replanning";
        return;
    }
    int ms = robot_config.step_ms;
    nav.ticket = motion_submit(FORWARD, robot_config.nav_speed, ms, SRC_NAV, -1);
    if (nav.ticket) {
        nav.issued = FORWARD;
        nav.deadline = now + (int64_t)(ms + 750) * 1000;
        nav.message = "moving";
    } else {
        end("motion_refused", false);
    }
}

static void nav_task(void *unused)
{
    (void)unused;
    while (true) {
        xSemaphoreTake(lock, portMAX_DELAY);
        planner_tick(esp_timer_get_time());
        xSemaphoreGive(lock);
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

static void load(void)
{
    nvs_handle_t nvs;
    if (nvs_open("rover_map", NVS_READONLY, &nvs) == ESP_OK) {
        size_t size = sizeof(saved_map);
        esp_err_t result = nvs_get_blob(nvs, "grid", &saved_map, &size);
        nvs_close(nvs);
        if (result == ESP_OK && size == sizeof(saved_map) && saved_map.version == MAP_VERSION &&
            saved_map.cell_cm == robot_config.cell_cm) return;
    }
    memset(&saved_map, 0, sizeof(saved_map));
    saved_map.version = MAP_VERSION;
    saved_map.cell_cm = robot_config.cell_cm;
}

bool nav_save(void)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    bool ok = false;
    nvs_handle_t nvs;
    if (nav.mode == NAV_IDLE && nvs_open("rover_map", NVS_READWRITE, &nvs) == ESP_OK) {
        ok = nvs_set_blob(nvs, "grid", &saved_map, sizeof(saved_map)) == ESP_OK && nvs_commit(nvs) == ESP_OK;
        nvs_close(nvs);
    }
    xSemaphoreGive(lock);
    return ok;
}

cJSON *nav_json(void)
{
    cJSON *json = cJSON_CreateObject();
    if (!json) return NULL;
    xSemaphoreTake(lock, portMAX_DELAY);
    cJSON_AddStringToObject(json, "mode", MODE_NAMES[nav.mode]);
    cJSON_AddStringToObject(json, "message", nav.message);
    cJSON_AddBoolToObject(json, "pose", nav.pose);
    cJSON_AddNumberToObject(json, "x", nav.pos.x);
    cJSON_AddNumberToObject(json, "y", nav.pos.y);
    cJSON_AddNumberToObject(json, "heading", nav.heading);
    cJSON_AddNumberToObject(json, "goal_x", nav.target.x);
    cJSON_AddNumberToObject(json, "goal_y", nav.target.y);
    cJSON_AddNumberToObject(json, "cell_cm", saved_map.cell_cm);
    cJSON *rows = cJSON_AddArrayToObject(json, "cells");
    for (int y = 0; y < GRID_SIDE; ++y) {
        char row[GRID_SIDE + 1] = {0};
        for (int x = 0; x < GRID_SIDE; ++x) {
            cell_t kind = grid_get(&saved_map.grid, (cell_pos_t){x,y});
            row[x] = kind == CELL_FREE ? '.' : kind == CELL_BLOCKED ? '#' : '?';
        }
        cJSON_AddItemToArray(rows, cJSON_CreateString(row));
    }
    cJSON *points = cJSON_AddArrayToObject(json, "points");
    for (int i = 0; i < POINTS; ++i) {
        if (!saved_map.points[i].name[0]) continue;
        cJSON *item = cJSON_CreateObject();
        if (!item) continue;
        cJSON_AddStringToObject(item, "name", saved_map.points[i].name);
        cJSON_AddNumberToObject(item, "x", saved_map.points[i].x);
        cJSON_AddNumberToObject(item, "y", saved_map.points[i].y);
        cJSON_AddItemToArray(points, item);
    }
    xSemaphoreGive(lock);
    return json;
}

void nav_prompt_context(char *buffer, size_t length)
{
    if (!length) return;
    xSemaphoreTake(lock, portMAX_DELAY);
    int used = snprintf(buffer, length, "地图状态=%s，当前位置=%s，已登记地点：起点",
                        MODE_NAMES[nav.mode], nav.pose ? "已校准（估计）" : "待校准");
    for (int i = 0; i < POINTS && used >= 0 && (size_t)used < length; ++i) {
        if (saved_map.points[i].name[0]) {
            used += snprintf(buffer + used, length - used, "、%s", saved_map.points[i].name);
        }
    }
    xSemaphoreGive(lock);
}

void nav_start(void)
{
    lock = xSemaphoreCreateMutex();
    assert(lock);
    load();
    nav.pos = HOME;
    nav.target = HOME;
    map_task_ready = xTaskCreate(nav_task, "planner", 5632, NULL, 4, NULL) == pdPASS;
    if (!map_task_ready) ESP_LOGE("nav", "Navigation task unavailable");
}
