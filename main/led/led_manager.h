#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef enum {
    LED_PATTERN_IDLE,
    LED_PATTERN_DISCONNECTED,
    LED_PATTERN_CONNECTED,
    LED_PATTERN_PAIRING,
    LED_PATTERN_LOW_BATTERY,
    LED_PATTERN_DEBUG_FLASH,
    LED_PATTERN_POWER_OFF,
    LED_PATTERN_TEST,
} led_pattern_id_t;

esp_err_t led_manager_init(void);
esp_err_t led_set_pattern(led_pattern_id_t pattern);
esp_err_t led_clear_pattern(led_pattern_id_t pattern);
led_pattern_id_t led_get_current_pattern(void);