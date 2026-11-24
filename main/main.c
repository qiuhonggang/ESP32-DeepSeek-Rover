#include "robot.h"
#include "nav.h"
#include "harness.h"
#include "esp_log.h"
#include "nvs_flash.h"

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    network_load_config();
    motion_start();
    nav_start();
    harness_start();
    cloud_start();
    network_start();
    if (!audio_start()) {
        ESP_LOGE("rover", "Audio device initialization failed; use the chassis console to isolate wiring.");
    }
}
