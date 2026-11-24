#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
    PIN_BCLK = 4, PIN_WS = 5, PIN_MIC = 6, PIN_SPEAKER = 7,
    PIN_TRIG = 8, PIN_ECHO = 9,
    PIN_LF_A = 10, PIN_LF_B = 11, PIN_RF_A = 12, PIN_RF_B = 13,
    PIN_LR_A = 14, PIN_LR_B = 15, PIN_SLEEP = 16, PIN_RR_A = 17, PIN_RR_B = 18,
    SAMPLE_RATE = 16000, MAX_FRAME = 512, MAX_WIRE_BYTES = 2048,
};

typedef enum { SRC_NAV = 1, SRC_CLOUD, SRC_WEB } motion_source_t;
typedef enum { STOP, FORWARD, BACK, LEFT, RIGHT, WIGGLE } action_t;
typedef enum { DIR_UNKNOWN, DIR_LEFT, DIR_CENTER, DIR_RIGHT } direction_t;
typedef enum { MOTION_IDLE, MOTION_RUNNING, MOTION_DONE, MOTION_ABORTED } motion_result_t;

typedef struct {
    char ssid[33], password[65], api_key[161], console_key[25], voice[10], profile[301];
    int stop_cm, speed, nav_speed, step_ms, turn_left_ms, turn_right_ms, cell_cm;
    int polarity[4];
    int vad_rms, barge_rms, mic_spacing_mm, volume, ref_delay;
    bool history, barge;
    uint32_t wake_choice;
} robot_config_t;

typedef struct {
    int cm;
    bool valid, blocked, armed, moving;
    uint32_t measurement, ticket;
    motion_result_t result;
    action_t action;
} motion_status_t;

typedef struct {
    int left_rms, right_rms, speech_rms;
    direction_t direction;
    bool listening, speaking, aec, wake_ready;
} audio_status_t;

typedef enum { UP_WAKE, UP_AUDIO, UP_END, UP_ABORT, UP_INTERRUPT, UP_PLAY_DONE, UP_PLAY_ABORT } uplink_type_t;
typedef struct {
    uplink_type_t type;
    uint32_t id;
    uint16_t bytes;
    direction_t direction;
    int16_t pcm[MAX_FRAME];
} uplink_t;

extern robot_config_t robot_config;

void motion_start(void);
uint32_t motion_submit(action_t action, int speed, int duration_ms, motion_source_t source, int wheel);
bool motion_arm(void);
void motion_stop(bool disarm);
motion_status_t motion_status(void);
action_t action_from_name(const char *name);
const char *action_name(action_t action);

typedef struct {
    const char *model;
    const char *phrase;
    const char *aliases[3];
} wake_option_t;
extern const wake_option_t wake_options[];
extern const unsigned wake_option_count;
const wake_option_t *audio_selected_wake(void);
bool audio_start(void);
audio_status_t audio_status(void);
bool audio_begin_play(uint32_t id);
bool audio_push_play(const void *data, size_t size);
void audio_end_play(uint32_t id);
void audio_interrupt(void);
void audio_cancel_turn(uint32_t id);
bool audio_turn_is_current(uint32_t id);
bool audio_test_tone(void);
bool audio_listen_once(void);
const char *direction_name(direction_t direction);
int pcm_rms(const int16_t *pcm, size_t count);
direction_t estimate_direction(const int16_t *left, const int16_t *right, int count, int spacing_mm);

void network_load_config(void);
void network_start(void);
bool network_ready(void);
bool network_wifi_up(void);

void cloud_start(void);
bool cloud_uplink(const uplink_t *packet);
