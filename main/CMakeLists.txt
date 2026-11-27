idf_component_register(SRCS "main.c" "motion.c" "audio.c" "acoustics.c"
                            "wake_words.c" "network.c" "wifi_order.c" "cloud.c" "harness.c" "audio_wire.c"
                            "nav.c" "grid.c" "intent.c" "local_tts.c" "tts_text.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_driver_i2s esp_driver_ledc esp_timer esp_wifi
                  esp_event esp_netif esp_http_server esp_http_client nvs_flash lwip
                  esp_hw_support bootloader_support esp_partition mbedtls esp-sr cjson
    EMBED_TXTFILES "console.html"
    EMBED_FILES "cues/wake_1.pcm" "cues/wake_2.pcm" "cues/wake_3.pcm" "cues/wake_4.pcm"
                "cues/reply_stop.pcm" "cues/reply_lock.pcm" "cues/reply_soft.pcm" "cues/reply_loud.pcm"
                "cues/reply_volume.pcm" "cues/reply_slow.pcm" "cues/reply_fast.pcm"
                "cues/reply_normal.pcm" "cues/reply_bye.pcm")

idf_component_get_property(esp_sr_dir espressif__esp-sr COMPONENT_DIR)
idf_build_get_property(build_dir BUILD_DIR)
set(xiaole_voice "${esp_sr_dir}/esp-tts/esp_tts_chinese/esp_tts_voice_data_xiaole.dat")
set(voice_image "${build_dir}/voice_data.bin")
configure_file("${xiaole_voice}" "${voice_image}" COPYONLY)
file(SIZE "${voice_image}" voice_bytes)
target_compile_definitions(${COMPONENT_LIB} PRIVATE XIAOLE_VOICE_BYTES=${voice_bytes})
esptool_py_flash_to_partition(flash "voice_data" "${voice_image}")
