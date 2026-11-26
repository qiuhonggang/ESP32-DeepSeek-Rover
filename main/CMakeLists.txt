idf_component_register(SRCS "main.c" "motion.c" "audio.c" "acoustics.c"
                            "wake_words.c" "network.c" "wifi_order.c" "cloud.c" "harness.c" "audio_wire.c"
                            "nav.c" "grid.c" "intent.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_driver_i2s esp_driver_ledc esp_timer esp_wifi
                  esp_event esp_netif esp_http_server esp_http_client nvs_flash lwip
                  esp_hw_support bootloader_support mbedtls esp-sr cjson
    EMBED_TXTFILES "console.html"
    EMBED_FILES "cues/wake_1.pcm" "cues/wake_2.pcm" "cues/wake_3.pcm" "cues/wake_4.pcm"
                "cues/reply_stop.pcm" "cues/reply_lock.pcm" "cues/reply_soft.pcm" "cues/reply_loud.pcm"
                "cues/reply_volume.pcm" "cues/reply_slow.pcm" "cues/reply_fast.pcm"
                "cues/reply_normal.pcm" "cues/reply_bye.pcm")
