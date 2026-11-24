#include "robot.h"
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
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
    int speed, duration_ms, wheel;
    int64_t started, until;
} command;
static int64_t brake_until, armed_until, last_measurement;
static int64_t echo_rise, echo_width;
static bool echo_open, initialized;
static TaskHandle_t sonar_handle;

static bool shift_allowed(action_t action, int64_t now)
{
    bool fresh = status.valid && now - last_measurement < 170000;
    if (action == FORWARD) return fresh && !status.blocked;
    if (action == LEFT || action == RIGHT || action == WIGGLE) {
        return !fresh || status.cm > 10;
    }
    return true;
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
    int64_t age_us = esp_timer_get_time() - last_measurement;
    if (last_measurement) copy.sample_age_ms = (int)(age_us / 1000);
    if (age_us >= 170000) {
        copy.valid = false;
        copy.blocked = true;
        copy.cm = -1;
    }
    taskEXIT_CRITICAL(&guard);
    return copy;
}

static void finish_locked(motion_result_t result, int64_t now)
{
    if (status.result == MOTION_RUNNING) status.result = result;
    if (status.moving) brake_until = now + 100000;
    command.action = STOP;
    status.action = STOP;
}

void motion_stop(bool disarm)
{
    taskENTER_CRITICAL(&guard);
    finish_locked(MOTION_ABORTED, esp_timer_get_time());
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
    bool allowed = initialized && status.result != MOTION_RUNNING &&
                   now >= brake_until && shift_allowed(FORWARD, now);
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
        !status.moving && now >= brake_until && shift_allowed(action, now)) {
        command.action = action;
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
        status.echo_us = (int)echo_width;
        status.valid = echo_width >= 100 && echo_width <= 23500;
        status.cm = status.valid ? (int)((echo_width + 29) / 58) : -1;
        ++status.measurement;
        last_measurement = esp_timer_get_time();
        if (!status.valid || status.cm <= robot_config.stop_cm) {
            status.blocked = true;
            clear_samples = 0;
        } else if (status.cm >= robot_config.stop_cm + 5) {
            if (clear_samples < 2) ++clear_samples;
            if (clear_samples >= 2) status.blocked = false;
        } else {
            clear_samples = 0;
        }
        taskEXIT_CRITICAL(&guard);
        vTaskDelayUntil(&next, pdMS_TO_TICKS(65));
    }
}

static void pair(int motor, int speed, bool brake)
{
    int first = speed > 0 ? speed : 0;
    int second = speed < 0 ? -speed : 0;
    if (brake) first = second = 255;
    int channel = motor * 2;
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, channel, first, 0);
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, channel + 1, second, 0);
}

static void motor_task(void *unused)
{
    (void)unused;
    int previous[4] = {0};
    bool last_brake = false, last_enable = false;
    TickType_t next = xTaskGetTickCount();

    while (true) {
        int64_t now = esp_timer_get_time();
        int output[4] = {0};
        taskENTER_CRITICAL(&guard);
        if (status.armed && now >= armed_until) status.armed = false;
        if (command.action != STOP) {
            if (!status.armed || !shift_allowed(command.action, now)) {
                finish_locked(MOTION_ABORTED, now);
            } else {
                if (!command.started) {
                    command.started = now;
                    command.until = now + (int64_t)command.duration_ms * 1000;
                }
                if (now >= command.until) finish_locked(MOTION_DONE, now);
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
        bool enabled = status.armed || brake;
        taskEXIT_CRITICAL(&guard);

        if (enabled && !last_enable) gpio_set_level(PIN_SLEEP, 1);
        for (int i = 0; i < 4; ++i) {
            if (output[i] != previous[i] || brake != last_brake) pair(i, output[i], brake);
            previous[i] = output[i];
        }
        if (!enabled && last_enable) gpio_set_level(PIN_SLEEP, 0);
        last_enable = enabled;
        last_brake = brake;
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
        .timer_num = LEDC_TIMER_0, .freq_hz = 18000, .clk_cfg = LEDC_AUTO_CLK,
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
