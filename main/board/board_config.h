#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t board_config_validate(void);
esp_err_t board_config_init(void);
const char *board_name(void);
i2c_master_bus_handle_t board_i2c_bus(void);
bool board_button_pressed(void);