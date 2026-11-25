#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "ble_batch_packets.h"

enum {
    GATT_STREAM_BATTERY = BLE_BATCH_KIND_STATUS + 1,
    GATT_STREAM_COUNT = GATT_STREAM_BATTERY,
};

int gatt_server_init(void);
void gatt_set_subscription(uint16_t attr_handle, bool enabled);
void gatt_clear_subscriptions(void);
uint16_t gatt_stream_handle(unsigned kind);
uint32_t gatt_stream_epoch(unsigned kind);
bool gatt_stream_subscribed(unsigned kind);
