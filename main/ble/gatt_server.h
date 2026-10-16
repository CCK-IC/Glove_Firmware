#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "host/ble_gatt.h"

// extern uint16_t gatt_imu_handle;
extern uint16_t gatt_imu_raw_handle;
// extern uint16_t gatt_aux_handle;
extern uint16_t gatt_status_handle;
extern uint16_t gatt_control_handle;
extern uint16_t gatt_battery_handle;

int gatt_server_init(void);
void gatt_server_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);

void gatt_set_subscription(uint16_t attr_handle, bool enabled);
bool gatt_imu_subscribed(void);
bool gatt_imu_raw_subscribed(void);
bool gatt_status_subscribed(void);
bool gatt_battery_subscribed(void);