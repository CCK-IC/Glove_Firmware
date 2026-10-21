idf_component_register(
    SRCS
        "app_main.c"
        "app_events.c"
        "system_state.c"
        "board/board_config.c"
        "ble/ble_packets.c"
        "ble/gatt_server.c"
        "ble/ble_manager.c"
        "imu/imu_manager.c"
        "adc/adc_manager.c"
        "battery/battery_manager.c"
        "button/button_manager.c"
        "led/led_manager.c"
        "power/power_manager.c"
        "diagnostics/diagnostics.c"
    INCLUDE_DIRS
        "."
        "board"
        "ble"
        "adc"
        "imu"
        "battery"
        "button"
        "led"
        "power"
        "diagnostics"
    PRIV_REQUIRES
        bt
        nvs_flash
        esp_timer
        driver
        esp_adc
        esp_system
        esp_common
        freertos
        log
        icm20948
)

if(CONFIG_IMU_DMP_PROFILE_APPLICATION_OVERRIDE)
    if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/imu/imu_dmp_application_override.c")
        message(FATAL_ERROR
            "Application DMP override selected, but "
            "imu/imu_dmp_application_override.c was not supplied. "
            "A complete validated profile is required.")
    endif()

    target_sources(${COMPONENT_LIB} PRIVATE
        "imu/imu_dmp_application_override.c")
endif()