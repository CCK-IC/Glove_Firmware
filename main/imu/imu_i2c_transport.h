#pragma once

#include "driver/i2c_master.h"
#include "icm20948.h"

typedef struct {
    i2c_master_dev_handle_t device;
    esp_err_t first_error;
    uint8_t failed_register;
    bool failed_write;

    /* The driver's serif pointer is retained for the sensor's lifetime. */
    icm20948_serif_t serif;
    icm20948_device_t *sensor;
    int timeout_ms;
} imu_i2c_transport_t;

/* Swap the serif on an already configured sensor with an attached ESP-IDF
 * device. Preserve the driver's configured state and register caches. Serialize
 * calls with the device lock. */
void imu_i2c_attach(imu_i2c_transport_t *transport, icm20948_device_t *sensor);

/* Before a logical driver operation, clear its error and set a finite timeout
 * for each transfer. After the first failure, the rest of the operation fails
 * without issuing another physical transfer. */
void imu_i2c_begin(imu_i2c_transport_t *transport, int timeout_ms);
