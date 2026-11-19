#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "ble_packets.h"

#define BLE_SAMPLE_QUEUE_DEPTH 64U

/* Kinds match the v3 wire format. */
enum { BLE_BUFFER_RAW = 1, BLE_BUFFER_DMP = 2, BLE_BUFFER_AUX = 3 };

typedef struct {
    size_t count;
    uint32_t epoch;
    uint32_t oldest_timestamp_ms;
    bool enabled;
} ble_sample_buffer_info_t;

esp_err_t ble_sample_buffer_init(void);
void ble_sample_buffer_set_enabled(unsigned kind, bool enabled);
void ble_sample_buffer_disable_all(void);
void ble_sample_buffer_invalidate(unsigned kind);
uint32_t ble_sample_buffer_epoch(unsigned kind);
ble_sample_buffer_info_t ble_sample_buffer_info(unsigned kind);
void ble_sample_buffer_note_dropped(size_t count);

/* Epochs fence in-flight acquisitions across subscribe/reset/disconnect. */
void ble_sample_buffer_publish_raw(const imu_raw_sample_t *sample, uint32_t epoch);
void ble_sample_buffer_publish_dmp(const imu_dmp_sample_t *sample, uint32_t epoch);
void ble_sample_buffer_publish_aux(const adc_sample_t *sample, uint32_t epoch);
size_t ble_sample_buffer_take_raw(imu_raw_sample_t *out, size_t limit, uint32_t *epoch);
size_t ble_sample_buffer_take_dmp(imu_dmp_sample_t *out, size_t limit, uint32_t *epoch);
size_t ble_sample_buffer_take_aux(adc_sample_t *out, size_t limit, uint32_t *epoch);
