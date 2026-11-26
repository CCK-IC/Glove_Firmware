#pragma once

#include "icm20948.h"
#include "ble/ble_packets.h"

#define IMU_MAG_FRESHNESS_US 100000ULL
#define IMU_MAG_POLL_PERIOD_US 10000ULL
#define IMU_MAG_SLOW_FRAME_TIMEOUT_US 35000U

typedef struct {
    int16_t x, y, z;
    uint64_t updated_at_us; /* Last complete measurement, including overflow. */
    uint64_t next_poll_us;
    uint32_t frame_timeout_us; /* Zero selects the 20 ms normal-clock budget. */
    bool valid;
    bool overflow;
    bool needs_release;
    bool needs_recovery;
} imu_magnetometer_t;

/* Call with the IMU device lock held. DMP setup leaves AK09916 powered down;
 * its profile then owns SLV0/SLV1. Raw mode owns controlled SLV4 reads. */
icm20948_status_e imu_magnetometer_configure(icm20948_device_t *device,
                                            imu_magnetometer_t *mag, bool dmp);
icm20948_status_e imu_magnetometer_poll(icm20948_device_t *device,
                                       imu_magnetometer_t *mag, uint64_t now_us);
/* DMP uses its FIFO compass output instead of the raw auxiliary layout. */
void imu_magnetometer_update(imu_magnetometer_t *mag, int16_t x, int16_t y,
                             int16_t z, uint64_t now_us);
void imu_magnetometer_invalidate(imu_magnetometer_t *mag);
void imu_magnetometer_apply(const imu_magnetometer_t *mag,
                            imu_raw_sample_t *sample, uint64_t now_us);
