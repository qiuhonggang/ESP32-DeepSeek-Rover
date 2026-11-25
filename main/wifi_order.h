#pragma once
#include <stdbool.h>
#include <stdint.h>

enum { WIFI_PROFILE_LIMIT = 5, WIFI_SSID_BYTES = 32, WIFI_PASSWORD_BYTES = 64 };

typedef struct {
    char ssid[WIFI_SSID_BYTES + 1];
    char password[WIFI_PASSWORD_BYTES + 1];
} wifi_profile_t;

typedef struct {
    uint32_t count;
    wifi_profile_t entries[WIFI_PROFILE_LIMIT];
} wifi_profiles_t;

// A missing password retains the saved password for the exact SSID.
bool wifi_profiles_add(wifi_profiles_t *list, const wifi_profiles_t *saved,
                       const char *ssid, const char *password);

typedef struct {
    uint32_t next;
    int64_t attempt_until, retry_at, stable_at;
    bool trying, online, fallback_ap;
} wifi_order_t;

typedef enum { WIFI_ORDER_WAIT, WIFI_ORDER_CONNECT, WIFI_ORDER_RESET_STA } wifi_order_action_t;

wifi_order_action_t wifi_order_step(wifi_order_t *order, uint32_t count, int64_t now_ms,
                                  bool has_ip, bool got_ip, bool failed);
