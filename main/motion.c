#include "robot.h"
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *const ACTIONS[] = {
    "stop", "move_forward", "move_backward", "turn_left", "turn_right", "happy_wiggle",
};
static const int MOTOR_PINS[] = {
    PIN_LF_A, PIN_LF_B, PIN_RF_A, PIN_RF_B, PIN_LR_A, PIN_LR_B, PIN_RR_A, PIN_RR_B,
};
static portMUX_TYPE guard = portMUX_INITIALIZER_UNLOCKED;
static motion_status_t status = {.cm = -1, .sample_age_ms = -1, .wheel = -1, .blocked = true};
static struct {
    action_t action;
    motion_source_t source;
    int speed, duration_ms, wheel;
    int64_t started, until;
} command;
static int64_t brake_until, armed_until, last_measurement, last_valid_measurement;
static int64_t echo_rise, echo_width;
static bool echo_open, initialized;
static TaskHandle_t sonar_handle;

static const int MAX_SAMPLE_AGE_US = 170000;

int motion_approach_clearance_cm(action_t action)
{
    int cm = robot_config.stop_cm > 25 ? robot_config.stop_cm : 25;
    return action == FORWARD ? cm : cm + 5;
}

static bool shift_allowed(action_t action, motion_source_t source, int64_t now)
{
    bool fresh = status.valid && now - last_measurement < MAX_SAMPLE_AGE_US;
    if (source == SRC_APPROACH) return fresh && !status.blocked && status.cm > motion_approach_clearance_cm(action);
    if (action == FORWARD) return fresh && !status.blocked;
    if (action == LEFT || action == RIGHT || action == WIGGLE) {
        return !fresh || status.cm > 10;
    }
    return true;
}

static bool single_wheel_forward_running(void)
{
    return command.action == FORWARD && command.wheel >= 0 && command.started != 0;
}

static bool waiting_for_single_wheel_echo(int64_t now)
{
    // A missing echo can retain a clear gate for one ping during a running dot.
    // Only the motor loop can use that gate without a valid latest sample.
    return single_wheel_forward_running() && !status.valid && !status.blocked &&
           status.echo_us == 0 && now - last_valid_measurement < MAX_SAMPLE_AGE_US;
}

action_t action_from_name(const char *name)
{
    if (name) {
        for (int i = STOP; i <= WIGGLE; ++i) {
            if (strcmp(name, ACTIONS[i]) == 0) return (action_t)i;
        }
    }
    return (action_t)-1;
}

const char *action_name(action_t action)
{
    return action >= STOP && action <= WIGGLE ? ACTIONS[action] : "unknown";
}

motion_status_t motion_status(void)
{
    taskENTER_CRITICAL(&guard);
    motion_status_t copy = status;
    int64_t now = esp_timer_get_time();
    copy.braking = now < brake_until;
    if (command.started && command.action != STOP) copy.elapsed_ms = (now - command.started) / 1000;
    int64_t age_us = now - last_measurement;
    if (last_measurement) copy.sample_age_ms = (int)(age_us / 1000);
    if (age_us >= MAX_SAMPLE_AGE_US) {
        copy.valid = false;
        copy.cm = -1;
    }
    if (!copy.valid) copy.blocked = true;
    taskEXIT_CRITICAL(&guard);
    return copy;
}

static void stop_locked(int64_t now)
{
    if (command.action != STOP && command.started) {
        status.elapsed_ms = (now - command.started) / 1000;
        brake_until = now + 100000;
    }
    command.action = STOP;
    status.action = STOP;
}

static void abort_locked(const char *reason, int64_t now)
{
    if (status.result == MOTION_RUNNING) {
        status.result = MOTION_ABORTED;
        status.stop_reason = reason;
    }
    stop_locked(now);
}

void motion_stop(bool disarm)
{
    taskENTER_CRITICAL(&guard);
    abort_locked(disarm ? "lock" : "stop", esp_timer_get_time());
    if (disarm) {
        status.armed = false;
        armed_until = 0;
    }
    taskEXIT_CRITICAL(&guard);
}

bool motion_arm(void)
{
    taskENTER_CRITICAL(&guard);
    int64_t now = esp_timer_get_time();
    bool allowed = initialized && !status.output_fault && status.result != MOTION_RUNNING &&
                   now >= brake_until && shift_allowed(FORWARD, SRC_WEB, now);
    if (allowed) {
        status.armed = true;
        armed_until = now + 30LL * 60 * 1000000;
    }
    taskEXIT_CRITICAL(&guard);
    return allowed;
}

uint32_t motion_submit(action_t action, int speed, int duration_ms, motion_source_t source, int wheel)
{
    if (action < FORWARD || action > WIGGLE || wheel < -1 || wheel > 3 ||
        (wheel >= 0 && source != SRC_WEB)) return 0;
    int maximum = action == FORWARD ? 700 : 390;
    if (source == SRC_NAV) maximum = action == FORWARD ? 1800 : 1700;
    if (source == SRC_APPROACH) {
        if (action != FORWARD && action != LEFT && action != RIGHT) return 0;
        maximum = action == FORWARD ? APPROACH_LIMIT_MS : 650;
        if (speed > APPROACH_MAX_PWM) speed = APPROACH_MAX_PWM;
    }
    if (wheel >= 0) maximum = 240;
    if (action == BACK && maximum > 250) maximum = 250;
    if (speed < 65) speed = 65;
    if (speed > robot_config.speed) speed = robot_config.speed;
    if (action == BACK && speed > 115) speed = 115;
    if (duration_ms < 80) duration_ms = 80;
    if (duration_ms > maximum) duration_ms = maximum;
    int64_t now = esp_timer_get_time();
    uint32_t ticket = 0;

    taskENTER_CRITICAL(&guard);
    if (status.armed && now < armed_until && status.result != MOTION_RUNNING &&
        !status.moving && now >= brake_until && shift_allowed(action, source, now)) {
        command.action = action;
        command.source = source;
        command.speed = speed;
        command.wheel = wheel;
        command.duration_ms = duration_ms;
        command.started = 0;
        command.until = 0;
        if (++status.ticket == 0) ++status.ticket;
        status.result = MOTION_RUNNING;
        status.action = action;
        status.last_action = action;
        status.wheel = wheel;
        status.pwm = speed;
        status.duration_ms = duration_ms;
        status.elapsed_ms = 0;
        status.stop_reason = NULL;
        ticket = status.ticket;
    }
    taskEXIT_CRITICAL(&guard);
    return ticket;
}

static void echo_isr(void *unused)
{
    (void)unused;
    int level = gpio_get_level(PIN_ECHO);
    int64_t now = esp_timer_get_time();
    bool captured = false;
    portENTER_CRITICAL_ISR(&guard);
    if (echo_open && level) echo_rise = now;
    if (echo_open && !level && echo_rise != 0) {
        echo_width = now - echo_rise;
        echo_open = false;
        captured = true;
    }
    portEXIT_CRITICAL_ISR(&guard);
    if (captured && sonar_handle) {
        BaseType_t higher = pdFALSE;
        vTaskNotifyGiveFromISR(sonar_handle, &higher);
        if (higher) portYIELD_FROM_ISR();
    }
}

static void record_sonar_locked(int width_us, int64_t now, int *clear_samples)
{
    int stop_cm = robot_config.stop_cm;
    bool retry = width_us == 0 && single_wheel_forward_running() && status.valid &&
                 !status.blocked && status.cm >= stop_cm + 5 &&
                 now - last_valid_measurement < MAX_SAMPLE_AGE_US;
    status.echo_us = width_us;
    status.valid = width_us >= 100 && width_us <= 23500;
    status.cm = status.valid ? (width_us + 29) / 58 : -1;
    ++status.measurement;
    last_measurement = now;
    if (status.valid) last_valid_measurement = now;

    if ((!status.valid && !retry) || (status.valid && status.cm <= stop_cm)) {
        status.blocked = true;
        *clear_samples = 0;
    } else if (status.cm >= stop_cm + 5) {
        if (*clear_samples < 2) ++*clear_samples;
        if (*clear_samples >= 2) status.blocked = false;
    } else {
        *clear_samples = 0;
    }
}

static void sonar_task(void *unused)
{
    (void)unused;
    int clear_samples = 0;
    sonar_handle = xTaskGetCurrentTaskHandle();
    TickType_t next = xTaskGetTickCount();
    while (true) {
        ulTaskNotifyTake(pdTRUE, 0);
        taskENTER_CRITICAL(&guard);
        echo_rise = echo_width = 0;
        echo_open = true;
        taskEXIT_CRITICAL(&guard);
        gpio_set_level(PIN_TRIG, 1);
        esp_rom_delay_us(12);
        gpio_set_level(PIN_TRIG, 0);
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(30));

        taskENTER_CRITICAL(&guard);
        echo_open = false;
        record_sonar_locked((int)echo_width, esp_timer_get_time(), &clear_samples);
        taskEXIT_CRITICAL(&guard);
        vTaskDelayUntil(&next, pdMS_TO_TICKS(65));
    }
}

static esp_err_t set_motor_duty(ledc_channel_t channel, uint32_t duty)
{
    // Only the motor task updates these channels, and it never uses LEDC fade.
    esp_err_t error = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    if (error != ESP_OK) return error;
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static esp_err_t pair(int motor, int speed, bool brake)
{
    int first = speed > 0 ? speed : 0;
    int second = speed < 0 ? -speed : 0;
    // 8-bit LEDC on S3: 256 is constant high. Slow decay drives for
    // |speed| ticks and brakes for the remainder of the 256-tick cycle.
    if (brake) {
        first = second = 256;
    } else if (robot_config.quiet_pwm && speed != 0) {
        first = 256 - second;
        second = speed > 0 ? 256 - speed : 256;
    }
    int physical_motor = (robot_config.swap_rear && motor >= 2) ? (motor ^ 1) : motor;
    int channel = physical_motor * 2;
    esp_err_t error = set_motor_duty(channel, first);
    if (error != ESP_OK) return error;
    return set_motor_duty(channel + 1, second);
}

static void motor_output_failed(int motor, esp_err_t error)
{
    gpio_set_level(PIN_SLEEP, 0);
    taskENTER_CRITICAL(&guard);
    stop_locked(esp_timer_get_time());
    status.armed = status.moving = false;
    status.output_fault = true;
    status.stop_reason = "pwm";
    status.result = MOTION_ABORTED;
    brake_until = armed_until = 0;
    taskEXIT_CRITICAL(&guard);

    for (int channel = 0; channel < sizeof(MOTOR_PINS) / sizeof(MOTOR_PINS[0]); ++channel) {
        esp_err_t stop_error = ledc_stop(LEDC_LOW_SPEED_MODE, channel, 0);
        if (stop_error != ESP_OK) {
            ESP_LOGE("motion", "Failed to stop PWM channel %d (%s)", channel, esp_err_to_name(stop_error));
        }
    }
    ESP_LOGE("motion", "PWM update failed for motor %d (%s). Chassis locked", motor, esp_err_to_name(error));
}

static void motor_task(void *unused)
{
    (void)unused;
    int previous[4] = {0};
    bool previous_brake[4] = {false}, last_enable = false;
    TickType_t next = xTaskGetTickCount();

    while (true) {
        int64_t now = esp_timer_get_time();
        int output[4] = {0};
        uint32_t finished_ticket = 0;
        taskENTER_CRITICAL(&guard);
        if (status.armed && now >= armed_until) status.armed = false;
        if (command.action != STOP) {
            if (!status.armed || (!shift_allowed(command.action, command.source, now) &&
                                  !waiting_for_single_wheel_echo(now))) {
                const char *reason = "obstacle";
                if (!status.armed) reason = "lock";
                else if (!last_measurement || now - last_measurement >= MAX_SAMPLE_AGE_US) reason = "stale";
                else if (!status.valid) reason = status.echo_us ? "echo_range" : "echo";
                abort_locked(reason, now);
            } else {
                if (!command.started) {
                    command.started = now;
                    command.until = now + (int64_t)command.duration_ms * 1000;
                }
                if (now >= command.until) {
                    finished_ticket = status.ticket;
                    stop_locked(now);
                }
            }
        }
        int left = 0, right = 0;
        switch (command.action) {
            case FORWARD: left = right = command.speed; break;
            case BACK: left = right = -command.speed; break;
            case LEFT: left = -command.speed; right = command.speed; break;
            case RIGHT: left = command.speed; right = -command.speed; break;
            case WIGGLE:
                left = ((now - command.started) / 140000) % 2 ? command.speed : -command.speed;
                right = -left;
                break;
            case STOP: break;
        }
        for (int i = 0; i < 4; ++i) {
            if (command.wheel == -1 || command.wheel == i) {
                output[i] = (i % 2 ? right : left) * robot_config.polarity[i];
            }
        }
        status.moving = command.action != STOP;
        bool brake = !status.moving && now < brake_until;
        int brake_wheel = command.wheel;
        bool enabled = status.armed || brake;
        taskEXIT_CRITICAL(&guard);

        if (enabled && !last_enable) gpio_set_level(PIN_SLEEP, 1);
        for (int i = 0; i < 4; ++i) {
            bool brake_this_wheel = brake && (brake_wheel < 0 || brake_wheel == i);
            if (output[i] == previous[i] && brake_this_wheel == previous_brake[i]) continue;
            esp_err_t error = pair(i, output[i], brake_this_wheel);
            if (error != ESP_OK) {
                motor_output_failed(i, error);
                vTaskDelete(NULL);
                return;
            }
            previous[i] = output[i];
            previous_brake[i] = brake_this_wheel;
        }
        if (!enabled && last_enable) gpio_set_level(PIN_SLEEP, 0);
        last_enable = enabled;
        if (finished_ticket) {
            taskENTER_CRITICAL(&guard);
            if (status.ticket == finished_ticket && status.result == MOTION_RUNNING) status.result = MOTION_DONE;
            taskEXIT_CRITICAL(&guard);
        }
        vTaskDelayUntil(&next, pdMS_TO_TICKS(10));
    }
}

void motion_start(void)
{
    const gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << PIN_TRIG) | (1ULL << PIN_SLEEP),
        .mode = GPIO_MODE_OUTPUT, .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&outputs));
    gpio_set_level(PIN_SLEEP, 0);
    const gpio_config_t input = {
        .pin_bit_mask = 1ULL << PIN_ECHO, .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&input));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ECHO, echo_isr, NULL));
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = robot_config.quiet_pwm ? 24000 : 1000, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    for (int i = 0; i < sizeof(MOTOR_PINS) / sizeof(MOTOR_PINS[0]); ++i) {
        const ledc_channel_config_t channel = {
            .gpio_num = MOTOR_PINS[i], .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = i, .timer_sel = LEDC_TIMER_0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }
    bool motor = xTaskCreatePinnedToCore(motor_task, "motor", 3072, NULL, 11, NULL, 1) == pdPASS;
    bool sonar = xTaskCreatePinnedToCore(sonar_task, "sonar", 3072, NULL, 10, NULL, 1) == pdPASS;
    taskENTER_CRITICAL(&guard);
    initialized = motor && sonar;
    taskEXIT_CRITICAL(&guard);
    if (!initialized) ESP_LOGE("motion", "Chassis tasks failed to start; outputs locked");
}
