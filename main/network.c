#include "robot.h"
#include "cloud.h"
#include "harness.h"
#include "nav.h"
#include <ctype.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "driver/gpio.h"
#include "bootloader_random.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

static const char *TAG = "network";
robot_config_t robot_config = {
    .voice = "stella", .stop_cm = 20, .speed = 145, .nav_speed = 103,
    .step_ms = 690, .turn_left_ms = 520, .turn_right_ms = 520, .cell_cm = 25,
    .polarity = {1, 1, 1, 1}, .vad_rms = 295, .barge_rms = 530,
    .mic_spacing_mm = 100, .volume = 28, .ref_delay = 0, .history = true, .barge = true,
};
static esp_netif_t *station;
static char ap_ssid[20];
static atomic_bool wifi_up, rebooting;
static SemaphoreHandle_t config_lock;
static EventGroupHandle_t wifi_events;
static bool sta_link;
enum { STA_FAILED = BIT0, STA_GOT_IP = BIT1, STA_STOPPED = BIT2 };
static const char STORE[] = "rover_cfg";
extern const char console_start[] asm("_binary_console_html_start");

typedef struct { const char *name; size_t field; int min, max; } tune_t;
#define TUNE(f, low, high) {#f, offsetof(robot_config_t, f), low, high}
static const tune_t settings[] = {
    TUNE(stop_cm, 15, 100), TUNE(speed, 80, 180), TUNE(nav_speed, 65, 180),
    TUNE(step_ms, 130, 1800), TUNE(turn_left_ms, 120, 1700), TUNE(turn_right_ms, 120, 1700),
    TUNE(cell_cm, 20, 45), TUNE(vad_rms, 50, 3500), TUNE(barge_rms, 80, 4500),
    TUNE(mic_spacing_mm, 40, 220), TUNE(volume, 0, 75), TUNE(ref_delay, 0, 3),
};
#undef TUNE

static esp_err_t save_config(const robot_config_t *config)
{
    nvs_handle_t nvs;
    esp_err_t result = nvs_open(STORE, NVS_READWRITE, &nvs);
    if (result != ESP_OK) return result;
    result = nvs_set_blob(nvs, "config-v4", config, sizeof(*config));
    if (result == ESP_OK) result = nvs_commit(nvs);
    nvs_close(nvs);
    return result;
}

void network_load_config(void)
{
    config_lock = xSemaphoreCreateMutex();
    assert(config_lock);
    bool migrated = false;
    nvs_handle_t nvs;
    if (nvs_open(STORE, NVS_READONLY, &nvs) == ESP_OK) {
        const struct { const char *key; size_t bytes; } versions[] = {
            {"config-v4", sizeof(robot_config_t)},
            {"config-v3", offsetof(robot_config_t, wifi_profiles)},
            {"config-v2", offsetof(robot_config_t, ap_password)},
            {"config-v1", offsetof(robot_config_t, wake_choice)},
        };
        for (size_t i = 0; i < sizeof(versions) / sizeof(versions[0]); ++i) {
            robot_config_t saved = robot_config;
            size_t length = versions[i].bytes;
            if (nvs_get_blob(nvs, versions[i].key, &saved, &length) != ESP_OK ||
                length != versions[i].bytes || saved.wifi_profiles.count > WIFI_PROFILE_LIMIT) continue;
            robot_config = saved;
            migrated = i > 0;
            break;
        }
        nvs_close(nvs);
    }
    if (migrated) {
        wifi_profiles_add(&robot_config.wifi_profiles, NULL, robot_config.legacy_ssid,
                          robot_config.legacy_password);
        memset(robot_config.legacy_ssid, 0, sizeof(robot_config.legacy_ssid));
        memset(robot_config.legacy_password, 0, sizeof(robot_config.legacy_password));
    }
    if (robot_config.wake_choice >= wake_option_count) robot_config.wake_choice = 0;
    if (!robot_config.console_key[0]) {
        uint32_t entropy[3];
        bootloader_random_enable();
        esp_fill_random(entropy, sizeof(entropy));
        bootloader_random_disable();
        snprintf(robot_config.console_key, sizeof(robot_config.console_key), "%08lx%08lx%08lx",
                 (unsigned long)entropy[0], (unsigned long)entropy[1], (unsigned long)entropy[2]);
        ESP_ERROR_CHECK(save_config(&robot_config));
    } else if (migrated && save_config(&robot_config) != ESP_OK) {
        ESP_LOGW(TAG, "Could not save migrated Wi-Fi configuration");
    }
}

bool network_change_volume(int amount, bool relative, int *applied)
{
    xSemaphoreTake(config_lock, portMAX_DELAY);
    int value = relative ? robot_config.volume + amount : amount;
    if (value < 0) value = 0;
    if (value > 75) value = 75;
    bool ok = !atomic_load(&rebooting);
    if (ok && value != robot_config.volume) {
        robot_config_t config = robot_config;
        config.volume = value;
        ok = save_config(&config) == ESP_OK;
        if (ok) {
            robot_config.volume = value;
            audio_set_volume(value);
        }
    }
    if (ok && applied) *applied = value;
    xSemaphoreGive(config_lock);
    return ok;
}

bool network_wifi_up(void) { return atomic_load(&wifi_up); }
static bool clock_ready(void) { return time(NULL) > 1700000000; }

bool network_ready(void)
{
    return robot_config.api_key[0] && network_wifi_up() && clock_ready() && !atomic_load(&rebooting);
}

static bool set_provisioning(bool enabled)
{
    return esp_wifi_set_mode(enabled ? WIFI_MODE_APSTA : WIFI_MODE_STA) == ESP_OK;
}

static bool reset_station(bool provisioning)
{
    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) != ESP_OK) return false;
    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
        xEventGroupClearBits(wifi_events, STA_STOPPED);
        if (esp_wifi_set_mode(provisioning ? WIFI_MODE_AP : WIFI_MODE_NULL) != ESP_OK) return false;
        EventBits_t stopped = xEventGroupWaitBits(wifi_events, STA_STOPPED, pdTRUE, pdFALSE,
                                                pdMS_TO_TICKS(1500));
        if (!(stopped & STA_STOPPED)) ESP_LOGW(TAG, "Station stop event timed out");
    }
    xEventGroupClearBits(wifi_events, STA_FAILED | STA_GOT_IP);
    return set_provisioning(provisioning);
}

static void connect_profile(const wifi_profile_t *profile, uint32_t priority)
{
    wifi_config_t config = {0};
    memcpy(config.sta.ssid, profile->ssid, strlen(profile->ssid));
    memcpy(config.sta.password, profile->password, strlen(profile->password));
    ESP_LOGI(TAG, "Trying Wi-Fi priority %lu", (unsigned long)priority);
    if (esp_wifi_set_config(WIFI_IF_STA, &config) != ESP_OK || esp_wifi_connect() != ESP_OK) {
        xEventGroupSetBits(wifi_events, STA_FAILED);
    }
}

static void connection_task(void *unused)
{
    (void)unused;
    const wifi_profiles_t *profiles = &robot_config.wifi_profiles;
    bool ap = profiles->count == 0;
    bool forced_setup = false;
    bool station_reset_needed = false;
    wifi_order_t order = {.fallback_ap = ap};
    int64_t pressed_at = 0;
    while (true) {
        int64_t now = esp_timer_get_time();
        if (gpio_get_level(PIN_BOOT)) {
            pressed_at = 0;
        } else if (!pressed_at) {
            pressed_at = now;
        } else if (!forced_setup && now - pressed_at >= 3000000) {
            forced_setup = true;
            nav_stop(true);
            audio_interrupt();
            ESP_LOGI(TAG, "BOOT requested provisioning at http://192.168.4.1");
        }
        if (!atomic_load(&rebooting)) {
            bool enable_ap = forced_setup || order.fallback_ap;
            if (station_reset_needed) {
                if (reset_station(enable_ap)) {
                    station_reset_needed = false;
                    ap = enable_ap;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            EventBits_t events = xEventGroupWaitBits(wifi_events, STA_FAILED | STA_GOT_IP,
                                                     pdTRUE, pdFALSE, 0);
            wifi_order_action_t action = wifi_order_step(&order, profiles->count, now / 1000,
                network_wifi_up(), events & STA_GOT_IP, events & STA_FAILED);
            if (action == WIFI_ORDER_CONNECT) {
                connect_profile(&profiles->entries[order.next - 1], order.next);
            } else if (action == WIFI_ORDER_RESET_STA) {
                station_reset_needed = !reset_station(ap);
            }
            enable_ap = forced_setup || order.fallback_ap;
            if (!station_reset_needed && ap != enable_ap && set_provisioning(enable_ap)) ap = enable_ap;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t event, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && event == WIFI_EVENT_AP_START) ESP_LOGI(TAG, "Provisioning AP active: %s", ap_ssid);
    if (base == WIFI_EVENT && event == WIFI_EVENT_STA_CONNECTED) sta_link = true;
    if (base == WIFI_EVENT && (event == WIFI_EVENT_STA_DISCONNECTED || event == WIFI_EVENT_STA_STOP)) {
        sta_link = false;
        if (atomic_exchange(&wifi_up, false)) {
            nav_stop(true);
            audio_network_lost();
        }
        xEventGroupSetBits(wifi_events, event == WIFI_EVENT_STA_STOP ? STA_STOPPED : STA_FAILED);
    }
    if (base == IP_EVENT && event == IP_EVENT_STA_GOT_IP && sta_link) {
        atomic_store(&wifi_up, true);
        xEventGroupSetBits(wifi_events, STA_GOT_IP);
        const ip_event_got_ip_t *ip = data;
        ESP_LOGI(TAG, "Rover console at http://" IPSTR, IP2STR(&ip->ip_info.ip));
        if (!esp_sntp_enabled()) {
            esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
            esp_sntp_setservername(0, "ntp.aliyun.com");
            esp_sntp_setservername(1, "pool.ntp.org");
            esp_sntp_init();
        }
    }
}

static bool authorize(httpd_req_t *request)
{
    char key[32];
    unsigned mismatch = 1;
    if (httpd_req_get_hdr_value_str(request, "X-Console-Key", key, sizeof(key)) == ESP_OK &&
        strlen(key) == strlen(robot_config.console_key)) {
        mismatch = 0;
        for (size_t i = 0; i < strlen(robot_config.console_key); ++i) {
            mismatch |= (unsigned char)key[i] ^ (unsigned char)robot_config.console_key[i];
        }
    }
    if (!mismatch) return true;
    httpd_resp_send_err(request, HTTPD_401_UNAUTHORIZED, "Invalid console key");
    return false;
}

static esp_err_t send_json(httpd_req_t *req, cJSON *json)
{
    char *rendered = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    if (!rendered) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t result = httpd_resp_sendstr(req, rendered);
    cJSON_free(rendered);
    return result;
}

static cJSON *read_json(httpd_req_t *req)
{
    if (req->content_len < 2 || req->content_len > 6144) return NULL;
    char *body = malloc(req->content_len + 1);
    if (!body) return NULL;
    size_t pos = 0;
    while (pos < req->content_len) {
        int bytes = httpd_req_recv(req, body + pos, req->content_len - pos);
        if (bytes <= 0) {
            free(body);
            return NULL;
        }
        pos += bytes;
    }
    body[pos] = 0;
    cJSON *json = cJSON_ParseWithOpts(body, NULL, true);
    free(body);
    if (cJSON_IsObject(json)) return json;
    cJSON_Delete(json);
    return NULL;
}

static const char *get_text(const cJSON *json, const char *key)
{
    return cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(json, key));
}

static int get_int(const cJSON *json, const char *key, int fallback)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(json, key);
    return cJSON_IsNumber(item) && item->valuedouble == item->valueint ? item->valueint : fallback;
}

static bool get_bool(const cJSON *object, const char *key)
{
    return cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(object, key));
}

static esp_err_t root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, console_start);
}

static esp_err_t get_status(httpd_req_t *req)
{
    if (!authorize(req)) return ESP_OK;
    cJSON *json = cJSON_CreateObject();
    if (json) {
        motion_status_t chassis = motion_status();
        audio_status_t audio = audio_status();
        esp_netif_ip_info_t ip = {0};
        esp_netif_get_ip_info(station, &ip);
        char address[20];
        snprintf(address, sizeof(address), IPSTR, IP2STR(&ip.ip));
        cJSON_AddStringToObject(json, "ip", address);
        wifi_ap_record_t access_point;
        char ssid[WIFI_SSID_BYTES + 1] = {0};
        if (network_wifi_up() && esp_wifi_sta_get_ap_info(&access_point) == ESP_OK) {
            memcpy(ssid, access_point.ssid, WIFI_SSID_BYTES);
        }
        cJSON_AddStringToObject(json, "wifi_ssid", ssid);
        cJSON_AddBoolToObject(json, "wifi", network_wifi_up());
        cJSON_AddBoolToObject(json, "clock", clock_ready());
        cJSON_AddBoolToObject(json, "key_set", robot_config.api_key[0]);
        cJSON_AddBoolToObject(json, "armed", chassis.armed);
        cJSON_AddBoolToObject(json, "moving", chassis.moving);
        cJSON_AddBoolToObject(json, "valid", chassis.valid);
        cJSON_AddBoolToObject(json, "blocked", chassis.blocked);
        cJSON_AddNumberToObject(json, "cm", chassis.cm);
        cJSON_AddStringToObject(json, "action", action_name(chassis.action));
        cJSON_AddNumberToObject(json, "measurement", chassis.measurement);
        cJSON_AddNumberToObject(json, "echo_us", chassis.echo_us);
        cJSON_AddNumberToObject(json, "sample_age_ms", chassis.sample_age_ms);
        cJSON_AddNumberToObject(json, "stop_cm", robot_config.stop_cm);
        cJSON *motor = cJSON_AddObjectToObject(json, "motor");
        const char *results[] = {"idle", "running", "done", "aborted"};
        cJSON_AddStringToObject(motor, "result", results[chassis.result]);
        cJSON_AddNumberToObject(motor, "ticket", chassis.ticket);
        cJSON_AddStringToObject(motor, "action", action_name(chassis.last_action));
        cJSON_AddNumberToObject(motor, "wheel", chassis.wheel);
        cJSON_AddNumberToObject(motor, "pwm", chassis.pwm);
        cJSON_AddNumberToObject(motor, "duration_ms", chassis.duration_ms);
        cJSON_AddNumberToObject(json, "left_rms", audio.left_rms);
        cJSON_AddNumberToObject(json, "right_rms", audio.right_rms);
        cJSON_AddNumberToObject(json, "speech_rms", audio.speech_rms);
        cJSON_AddNumberToObject(json, "i2s_rx_errors", audio.i2s_rx_errors);
        cJSON_AddNumberToObject(json, "i2s_tx_errors", audio.i2s_tx_errors);
        cJSON_AddNumberToObject(json, "playback_underruns", audio.playback_underruns);
        cJSON_AddStringToObject(json, "playback_result", playback_result_name(audio.playback_result));
        cJSON_AddNumberToObject(json, "wake_cue", audio.wake_cue);
        cJSON_AddNumberToObject(json, "volume", audio_volume());
        cJSON_AddStringToObject(json, "doa", direction_name(audio.direction));
        cJSON_AddBoolToObject(json, "listening", audio.listening);
        cJSON_AddNumberToObject(json, "followup_ms", audio.followup_ms);
        cJSON_AddBoolToObject(json, "speaking", audio.speaking);
        cJSON_AddBoolToObject(json, "aec", audio.aec);
        cJSON_AddBoolToObject(json, "wake_ready", audio.wake_ready);
        cJSON_AddStringToObject(json, "wake_word", audio_selected_wake()->phrase);
        cJSON_AddItemToObject(json, "cloud", cloud_json());
        cJSON_AddItemToObject(json, "nav", nav_json());
    }
    return send_json(req, json);
}

static int *tuning_value(robot_config_t *config, const tune_t *field)
{
    return (int *)((char *)config + field->field);
}

static esp_err_t get_config(httpd_req_t *req)
{
    if (!authorize(req)) return ESP_OK;
    cJSON *json = cJSON_CreateObject();
    if (json) {
        const wifi_profiles_t *profiles = &robot_config.wifi_profiles;
        cJSON_AddStringToObject(json, "ssid", profiles->count ? profiles->entries[0].ssid : "");
        cJSON_AddBoolToObject(json, "has_password", profiles->count && profiles->entries[0].password[0]);
        cJSON_AddNumberToObject(json, "wifi_limit", WIFI_PROFILE_LIMIT);
        cJSON *networks = cJSON_AddArrayToObject(json, "wifi_networks");
        for (uint32_t i = 0; i < profiles->count; ++i) {
            cJSON *item = cJSON_CreateObject();
            if (!item) break;
            cJSON_AddStringToObject(item, "ssid", profiles->entries[i].ssid);
            cJSON_AddBoolToObject(item, "has_password", profiles->entries[i].password[0]);
            if (!cJSON_AddItemToArray(networks, item)) cJSON_Delete(item);
        }
        cJSON_AddStringToObject(json, "voice", robot_config.voice);
        cJSON_AddNumberToObject(json, "wake_choice", robot_config.wake_choice);
        cJSON *words = cJSON_AddArrayToObject(json, "wake_options");
        for (unsigned i = 0; i < wake_option_count; ++i) {
            cJSON_AddItemToArray(words, cJSON_CreateString(wake_options[i].phrase));
        }
        cJSON_AddStringToObject(json, "profile", robot_config.profile);
        cJSON_AddBoolToObject(json, "history", robot_config.history);
        cJSON_AddBoolToObject(json, "barge", robot_config.barge);
        cJSON_AddBoolToObject(json, "has_api_key", robot_config.api_key[0]);
        cJSON_AddBoolToObject(json, "has_ap_password", robot_config.ap_password[0]);
        xSemaphoreTake(config_lock, portMAX_DELAY);
        for (size_t i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i) {
            cJSON_AddNumberToObject(json, settings[i].name, *tuning_value(&robot_config, &settings[i]));
        }
        xSemaphoreGive(config_lock);
        cJSON_AddItemToObject(json, "polarity", cJSON_CreateIntArray(robot_config.polarity, 4));
    }
    return send_json(req, json);
}

static bool read_field(const cJSON *json, const char *name, char *target, size_t length)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(json, name);
    if (!item) return true;
    const char *value = cJSON_GetStringValue(item);
    if (!value || strlen(value) >= length) return false;
    strlcpy(target, value, length);
    return true;
}

static bool token_valid(const char *token)
{
    for (; *token; ++token) {
        if (!isalnum((unsigned char)*token) && *token != '-' && *token != '_' && *token != '.') return false;
    }
    return true;
}

static bool apply_wifi_profiles(const cJSON *json, wifi_profiles_t *profiles)
{
    const cJSON *networks = cJSON_GetObjectItemCaseSensitive(json, "wifi_networks");
    wifi_profiles_t updated = {0};
    if (networks) {
        if (!cJSON_IsArray(networks) || cJSON_GetArraySize(networks) > WIFI_PROFILE_LIMIT ||
            cJSON_HasObjectItem(json, "ssid") || cJSON_HasObjectItem(json, "password")) return false;
        const cJSON *entry;
        cJSON_ArrayForEach(entry, networks) {
            const cJSON *secret = cJSON_GetObjectItemCaseSensitive(entry, "password");
            if (secret && !cJSON_IsString(secret)) return false;
            if (!wifi_profiles_add(&updated, profiles, get_text(entry, "ssid"),
                                   cJSON_GetStringValue(secret))) return false;
        }
    } else {
        // Older console clients can still update the first profile.
        if (!cJSON_HasObjectItem(json, "ssid") && !cJSON_HasObjectItem(json, "password")) return true;
        wifi_profile_t first = profiles->entries[0];
        if (!read_field(json, "ssid", first.ssid, sizeof(first.ssid)) ||
            !read_field(json, "password", first.password, sizeof(first.password))) return false;
        if (first.ssid[0] && !wifi_profiles_add(&updated, NULL, first.ssid, first.password)) return false;
        for (uint32_t i = 1; i < profiles->count; ++i) {
            const wifi_profile_t *profile = &profiles->entries[i];
            if (!wifi_profiles_add(&updated, NULL, profile->ssid, profile->password)) return false;
        }
    }
    *profiles = updated;
    return true;
}

static bool apply_config(cJSON *json, robot_config_t *next)
{
    if (!json ||
        !apply_wifi_profiles(json, &next->wifi_profiles) ||
        !read_field(json, "api_key", next->api_key, sizeof(next->api_key)) ||
        !read_field(json, "profile", next->profile, sizeof(next->profile)) ||
        !read_field(json, "ap_password", next->ap_password, sizeof(next->ap_password)) ||
        !read_field(json, "voice", next->voice, sizeof(next->voice))) return false;
    if (next->ap_password[0] && strlen(next->ap_password) < 8) return false;
    for (const unsigned char *c = (const unsigned char *)next->ap_password; *c; ++c) {
        if (*c < 32 || *c > 126) return false;
    }
    int wake_choice = get_int(json, "wake_choice",
                              cJSON_HasObjectItem(json, "wake_choice") ? -1 : (int)next->wake_choice);
    if (wake_choice < 0 || (unsigned)wake_choice >= wake_option_count) return false;
    next->wake_choice = (uint32_t)wake_choice;
    if (!token_valid(next->api_key) || (strcmp(next->voice, "stella") && strcmp(next->voice, "tide"))) return false;
    if (cJSON_HasObjectItem(json, "history")) next->history = get_bool(json, "history");
    if (cJSON_HasObjectItem(json, "barge")) next->barge = get_bool(json, "barge");
    for (int i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i) {
        const tune_t *field = &settings[i];
        int *value = tuning_value(next, field);
        if (cJSON_HasObjectItem(json, field->name)) *value = get_int(json, field->name, -1);
        if (*value < field->min || *value > field->max) return false;
    }
    const cJSON *wheels = cJSON_GetObjectItemCaseSensitive(json, "polarity");
    if (wheels) {
        if (!cJSON_IsArray(wheels) || cJSON_GetArraySize(wheels) != 4) return false;
        for (int i = 0; i < 4; ++i) {
            const cJSON *item = cJSON_GetArrayItem(wheels, i);
            if (!cJSON_IsNumber(item) || (item->valuedouble != 1 && item->valuedouble != -1)) return false;
            next->polarity[i] = item->valueint;
        }
    }
    return next->nav_speed <= next->speed;
}

static void restart_task(void *unused)
{
    (void)unused;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(850));
    esp_restart();
}

static esp_err_t post_config(httpd_req_t *req)
{
    if (!authorize(req)) return ESP_OK;
    cJSON *json = read_json(req);
    xSemaphoreTake(config_lock, portMAX_DELAY);
    if (atomic_load(&rebooting)) {
        xSemaphoreGive(config_lock);
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Already restarting");
    }
    robot_config_t next = robot_config;
    bool valid = apply_config(json, &next);
    cJSON_Delete(json);
    if (!valid) {
        xSemaphoreGive(config_lock);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Wi-Fi, API key or tuning value");
    }
    atomic_store(&rebooting, true);
    nav_stop(true);
    audio_interrupt();
    TaskHandle_t restart = NULL;
    if (xTaskCreate(restart_task, "restart", 2048, NULL, 3, &restart) != pdPASS ||
        save_config(&next) != ESP_OK) {
        if (restart) vTaskDelete(restart);
        atomic_store(&rebooting, false);
        xSemaphoreGive(config_lock);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Could not save or restart");
    }
    xTaskNotifyGive(restart);
    xSemaphoreGive(config_lock);
    return httpd_resp_sendstr(req, "saved");
}

static bool control(const cJSON *json)
{
    const char *cmd = get_text(json, "cmd");
    if (!cmd) return false;
    if (!strcmp(cmd, "emergency")) {
        audio_interrupt();
        nav_stop(true);
        return true;
    }
    if (atomic_load(&rebooting)) return false;
    if (!strcmp(cmd, "stop")) {
        audio_interrupt();
        nav_stop(false);
        return true;
    }
    if (!strcmp(cmd, "arm")) return motion_arm();
    if (!strcmp(cmd, "tone")) return audio_test_tone();
    if (!strcmp(cmd, "wake_feedback")) return audio_wake_feedback();
    if (!strcmp(cmd, "volume")) {
        int value = get_int(json, "value", -1);
        return value >= 0 && value <= 75 && network_change_volume(value, false, NULL);
    }
    if (!strcmp(cmd, "listen") || !strcmp(cmd, "debug_asr")) {
        return audio_listen_once(!strcmp(cmd, "debug_asr"));
    }
    if (!strcmp(cmd, "debug_chat") || !strcmp(cmd, "debug_tts") || !strcmp(cmd, "debug_stream")) {
        return cloud_debug_text(get_text(json, "text"), cmd);
    }
    if (!strcmp(cmd, "stop_audio")) {
        audio_interrupt();
        return true;
    }
    if (!strcmp(cmd, "calibrate")) return nav_calibrate(action_from_name(get_text(json, "action")));
    if (!strcmp(cmd, "origin")) return nav_set_origin(get_bool(json, "clear"), get_int(json, "heading", NORTH));
    if (!strcmp(cmd, "explore")) return nav_explore();
    if (!strcmp(cmd, "goto")) return nav_go((cell_pos_t){get_int(json, "x", -1), get_int(json, "y", -1)});
    if (!strcmp(cmd, "goto_name")) return nav_go_name(get_text(json, "name"));
    if (!strcmp(cmd, "mark")) return nav_mark(get_text(json, "name")) && nav_save();
    if (!strcmp(cmd, "save_map")) return nav_save();
    if (!strcmp(cmd, "clear_memory")) return harness_clear(get_bool(json, "include_note"));
    int wheel = get_int(json, "wheel", -1);
    return nav_manual(action_from_name(cmd), get_int(json, "speed", 105),
                      get_int(json, "duration_ms", 240), SRC_WEB, wheel) != 0;
}

static esp_err_t post_control(httpd_req_t *req)
{
    if (!authorize(req)) return ESP_OK;
    cJSON *json = read_json(req);
    bool accepted = control(json);
    cJSON_Delete(json);
    if (!accepted) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Action refused");
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t get_memory(httpd_req_t *req)
{
    if (!authorize(req)) return ESP_OK;
    return send_json(req, harness_json());
}

void network_start(void)
{
    const gpio_config_t button = {
        .pin_bit_mask = 1ULL << PIN_BOOT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button));
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    station = esp_netif_create_default_wifi_sta();
    assert(station && esp_netif_create_default_wifi_ap());
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_events = xEventGroupCreate();
    assert(wifi_events);
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));
    snprintf(ap_ssid, sizeof(ap_ssid), "DeepSeekRover-%02X%02X", mac[4], mac[5]);
    wifi_config_t ap = {.ap = {.max_connection = 3, .channel = 6, .authmode = WIFI_AUTH_WPA2_PSK}};
    strlcpy((char *)ap.ap.ssid, ap_ssid, sizeof(ap.ap.ssid));
    const char *ap_password = robot_config.ap_password[0] ? robot_config.ap_password : robot_config.console_key;
    strlcpy((char *)ap.ap.password, ap_password, sizeof(ap.ap.password));
    // Configure the AP while its interface is enabled, before any beacons are sent.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    if (robot_config.wifi_profiles.count) ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Setup AP %s (hold BOOT) / http://192.168.4.1 / console key: %s",
             ap_ssid, robot_config.console_key);

    httpd_handle_t server;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 6400;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 3;
    config.send_wait_timeout = 3;
    ESP_ERROR_CHECK(httpd_start(&server, &config));
    const httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = root},
        {.uri = "/api/status", .method = HTTP_GET, .handler = get_status},
        {.uri = "/api/config", .method = HTTP_GET, .handler = get_config},
        {.uri = "/api/config", .method = HTTP_POST, .handler = post_config},
        {.uri = "/api/control", .method = HTTP_POST, .handler = post_control},
        {.uri = "/api/memory", .method = HTTP_GET, .handler = get_memory},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &routes[i]));
    }
    assert(xTaskCreate(connection_task, "wifi-watch", 3072, NULL, 3, NULL) == pdPASS);
}
