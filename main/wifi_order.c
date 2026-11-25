#include "wifi_order.h"
#include <string.h>

bool wifi_profiles_add(wifi_profiles_t *list, const wifi_profiles_t *saved,
                       const char *ssid, const char *password)
{
    if (!ssid || !*ssid || strlen(ssid) > WIFI_SSID_BYTES || list->count >= WIFI_PROFILE_LIMIT) return false;
    for (uint32_t i = 0; i < list->count; ++i) {
        if (!strcmp(list->entries[i].ssid, ssid)) return false;
    }
    if (!password && saved) {
        for (uint32_t i = 0; i < saved->count; ++i) {
            if (!strcmp(saved->entries[i].ssid, ssid)) {
                password = saved->entries[i].password;
                break;
            }
        }
    }
    if (!password) return false;
    size_t length = strlen(password);
    if (length > WIFI_PASSWORD_BYTES || (length && length < 8)) return false;
    wifi_profile_t *profile = &list->entries[list->count++];
    strcpy(profile->ssid, ssid);
    strcpy(profile->password, password);
    return true;
}

wifi_order_action_t wifi_order_step(wifi_order_t *order, uint32_t count, int64_t now_ms,
                                  bool has_ip, bool got_ip, bool failed)
{
    if (has_ip) {
        if (!order->online) order->stable_at = now_ms + 6000;
        order->online = true;
        order->trying = false;
        order->next = 0;
        order->retry_at = 0;
        if (now_ms >= order->stable_at) order->fallback_ap = false;
        return WIFI_ORDER_WAIT;
    }
    if (order->online || got_ip) {
        order->online = false;
        order->trying = false;
        order->next = 0;
        order->retry_at = 0;
    }
    if (order->trying) {
        if (!failed && now_ms < order->attempt_until) return WIFI_ORDER_WAIT;
        order->trying = false;
        // A timed-out DHCP/association may still be active in the driver.
        if (!failed) return WIFI_ORDER_RESET_STA;
    }
    if (now_ms < order->retry_at) return WIFI_ORDER_WAIT;
    if (order->next == count) {
        order->fallback_ap = true;
        order->next = 0;
        order->retry_at = now_ms + 15000;
        return WIFI_ORDER_WAIT;
    }
    ++order->next;
    order->trying = true;
    order->attempt_until = now_ms + 12000;
    return WIFI_ORDER_CONNECT;
}
