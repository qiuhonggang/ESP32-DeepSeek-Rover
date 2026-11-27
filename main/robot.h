#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "wifi_order.h"

enum {
    PIN_BOOT = 0,
    // H2: sonar on pins 4–5, shared I2S on pins 6–9.
    PIN_TRIG = 1, PIN_ECHO = 2,
    PIN_BCLK = 42, PIN_WS = 41, PIN_MIC = 40, PIN_SPEAKER = 39,
    // H1: front on pins 8–11, rear on pins 16–19, shared enable on pin 20.
    PIN_LF_A = 15, PIN_LF_B = 16, PIN_RF_A = 17, PIN_RF_B = 18,
    PIN_LR_A = 10, PIN_LR_B = 11, PIN_RR_A = 12, PIN_RR_B = 13,
    PIN_SLEEP = 14,
    SAMPLE_RATE = 16000, MAX_FRAME = 512, MAX_WIRE_BYTES = 2048,
};

typedef enum { SRC_NAV = 1, SRC_CLOUD, SRC_WEB, SRC_APPROACH } motion_source_t;
typedef enum { STOP, FORWARD, BACK, LEFT, RIGHT, WIGGLE } action_t;
typedef enum { DIR_UNKNOWN, DIR_LEFT, DIR_CENTER, DIR_RIGHT } direction_t;
typedef enum { MOTION_IDLE, MOTION_RUNNING, MOTION_DONE, MOTION_ABORTED } motion_result_t;
enum { APPROACH_LIMIT_MS = 6000, APPROACH_MAX_PWM = 145 };
typedef enum {
    PLAYBACK_NONE, PLAYBACK_COMPLETE, PLAYBACK_WAKE_WORD, PLAYBACK_MANUAL,
    PLAYBACK_I2S_ERROR, PLAYBACK_TIMEOUT, PLAYBACK_NETWORK_LOST, PLAYBACK_CLOUD_ERROR, PLAYBACK_VOICE,
} playback_result_t;

typedef struct {
    char legacy_ssid[33], legacy_password[65];  // Keep the layout of NVS v1–v3.
    char api_key[161], console_key[25], voice[10], profile[301];
    int stop_cm, speed, nav_speed, step_ms, turn_left_ms, turn_right_ms, cell_cm;
    int polarity[4];
    int vad_rms, barge_rms, mic_spacing_mm, volume, ref_delay;
    bool history, barge;
    uint32_t wake_choice;
    char ap_password[64];
    wifi_profiles_t wifi_profiles;
    uint32_t tts_local;  // Appended to preserve the NVS v4 prefix.
    uint32_t swap_rear;  // Appended after the NVS v5 prefix.
    uint32_t quiet_pwm;  // Appended after the NVS v6 prefix.
} robot_config_t;

typedef struct {
    int cm;
    bool valid, blocked, armed, moving, braking, output_fault;
    uint32_t measurement, ticket;
    int echo_us, sample_age_ms, wheel, pwm, duration_ms, elapsed_ms;
    const char *stop_reason;
    motion_result_t result;
    action_t action, last_action;
} motion_status_t;

typedef struct {
    int left_rms, right_rms, speech_rms;
    uint32_t i2s_rx_errors, i2s_tx_errors, playback_underruns, barge_checks, wake_cue;
    int followup_ms;
    direction_t direction;
    playback_result_t playback_result;
    bool listening, speaking, aec, wake_ready;
} audio_status_t;

typedef enum { UP_WAKE, UP_AUDIO, UP_END, UP_ABORT, UP_INTERRUPT, UP_PLAY_BEGIN, UP_PLAY_DONE, UP_PLAY_ABORT } uplink_type_t;
typedef struct {
    uplink_type_t type;
    uint32_t id;
    uint32_t at_ms;
    uint16_t bytes;
    direction_t direction;
    bool asr_only;
    int16_t pcm[MAX_FRAME];
} uplink_t;

extern robot_config_t robot_config;

void motion_start(void);
uint32_t motion_submit(action_t action, int speed, int duration_ms, motion_source_t source, int wheel);
bool motion_arm(void);
void motion_stop(bool disarm);
motion_status_t motion_status(void);
int motion_approach_clearance_cm(action_t action);
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
void audio_set_volume(int volume);
int audio_volume(void);
bool audio_begin_play(uint32_t id, bool isolated);
bool audio_begin_local_play(uint32_t id);
bool audio_push_play(uint32_t id, const void *data, size_t size);
void audio_end_play(uint32_t id);
void audio_interrupt(void);
void audio_network_lost(void);
void audio_cancel_turn(uint32_t id);
bool audio_turn_is_current(uint32_t id);
void audio_offer_followup(uint32_t id);
void audio_end_dialog(uint32_t id);
bool audio_test_tone(void);
bool audio_wake_feedback(void);
bool audio_listen_once(bool asr_only);
uint32_t audio_begin_debug_turn(void);
const char *direction_name(direction_t direction);
const char *playback_result_name(playback_result_t result);
int pcm_rms(const int16_t *pcm, size_t count);
direction_t estimate_direction(const int16_t *left, const int16_t *right, int count, int spacing_mm);
direction_t speech_direction(const unsigned votes[DIR_RIGHT + 1]);

void network_load_config(void);
void network_start(void);
bool network_ready(void);
bool network_wifi_up(void);
bool network_change_volume(int amount, bool relative, int *applied);

void cloud_start(void);
bool cloud_uplink(const uplink_t *packet);
