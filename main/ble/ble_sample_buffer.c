#include "ble_sample_buffer.h"

#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include "system_state.h"

typedef struct {
    uint32_t sequence;
    uint32_t timestamp_ms;
} sample_prefix_t;

_Static_assert(sizeof(sample_prefix_t) == 8 && offsetof(sample_prefix_t, timestamp_ms) == 4, "sample prefix layout");
_Static_assert(offsetof(imu_raw_sample_t, sequence) == 0 && offsetof(imu_raw_sample_t, timestamp_ms) == 4, "raw prefix");
_Static_assert(offsetof(imu_dmp_sample_t, sequence) == 0 && offsetof(imu_dmp_sample_t, timestamp_ms) == 4, "DMP prefix");
_Static_assert(offsetof(adc_sample_t, sequence) == 0 && offsetof(adc_sample_t, timestamp_ms) == 4, "AUX prefix");

/* No BLE calls, allocation or sensor access occurs while these locks are held. */
typedef struct {
    SemaphoreHandle_t lock;
    uint8_t *storage;
    size_t item_size;
    size_t head;
    size_t count;
    uint32_t epoch;
    bool enabled;
} sample_ring_t;

static imu_raw_sample_t s_raw[BLE_SAMPLE_QUEUE_DEPTH];
#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY
static imu_dmp_sample_t s_dmp[BLE_SAMPLE_QUEUE_DEPTH];
#endif
static adc_sample_t s_aux[BLE_SAMPLE_QUEUE_DEPTH];
static sample_ring_t s_rings[BLE_BUFFER_AUX] = {
    [BLE_BUFFER_RAW - 1] = {.storage = (uint8_t *)s_raw, .item_size = sizeof(s_raw[0])},
#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY
    [BLE_BUFFER_DMP - 1] = {.storage = (uint8_t *)s_dmp, .item_size = sizeof(s_dmp[0])},
#endif
    [BLE_BUFFER_AUX - 1] = {.storage = (uint8_t *)s_aux, .item_size = sizeof(s_aux[0])},
};

static sample_ring_t *ring_for(unsigned kind)
{
    return kind >= BLE_BUFFER_RAW && kind <= BLE_BUFFER_AUX ? &s_rings[kind - 1] : NULL;
}

esp_err_t ble_sample_buffer_init(void)
{
    /* Application initialization is serialized, before acquisition starts. */
    for (unsigned i = 0; i < BLE_BUFFER_AUX; ++i) {
        if (!s_rings[i].storage || s_rings[i].lock) continue;
        s_rings[i].lock = xSemaphoreCreateMutex();
        if (!s_rings[i].lock) return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void clear_locked(sample_ring_t *ring)
{
    ring->head = ring->count = 0;
    ++ring->epoch;
}

void ble_sample_buffer_set_enabled(unsigned kind, bool enabled)
{
    sample_ring_t *ring = ring_for(kind);
    if (!ring || !ring->lock) return;
    xSemaphoreTake(ring->lock, portMAX_DELAY);
    if (ring->enabled != enabled) {
        clear_locked(ring);
        ring->enabled = enabled;
    }
    xSemaphoreGive(ring->lock);
}

void ble_sample_buffer_disable_all(void)
{
    for (unsigned kind = BLE_BUFFER_RAW; kind <= BLE_BUFFER_AUX; ++kind)
        ble_sample_buffer_set_enabled(kind, false);
}

void ble_sample_buffer_invalidate(unsigned kind)
{
    sample_ring_t *ring = ring_for(kind);
    if (!ring || !ring->lock) return;
    xSemaphoreTake(ring->lock, portMAX_DELAY);
    clear_locked(ring);
    xSemaphoreGive(ring->lock);
}

ble_sample_buffer_info_t ble_sample_buffer_info(unsigned kind)
{
    ble_sample_buffer_info_t info = {0};
    sample_ring_t *ring = ring_for(kind);
    if (!ring || !ring->lock) return info;
    xSemaphoreTake(ring->lock, portMAX_DELAY);
    info.count = ring->count;
    info.epoch = ring->epoch;
    info.enabled = ring->enabled;
    if (ring->count) {
        /* Every acquisition struct starts with sequence u32, timestamp_ms u32. */
        memcpy(&info.oldest_timestamp_ms,
               ring->storage + ring->head * ring->item_size + sizeof(uint32_t),
               sizeof(info.oldest_timestamp_ms));
    }
    xSemaphoreGive(ring->lock);
    return info;
}

uint32_t ble_sample_buffer_epoch(unsigned kind)
{
    sample_ring_t *ring = ring_for(kind);
    if (!ring || !ring->lock) return 0;
    xSemaphoreTake(ring->lock, portMAX_DELAY);
    uint32_t epoch = ring->epoch;
    xSemaphoreGive(ring->lock);
    return epoch;
}

void ble_sample_buffer_note_dropped(size_t count)
{
    for (size_t i = 0; i < count; ++i) SYSTEM_STATE_INCREMENT(dropped_sensor_samples);
}

static void publish(unsigned kind, const void *sample, uint32_t epoch)
{
    sample_ring_t *ring = ring_for(kind);
    if (!ring || !ring->lock || !sample) return;
    bool dropped = false;
    xSemaphoreTake(ring->lock, portMAX_DELAY);
    if (ring->enabled && epoch == ring->epoch) {
        if (ring->count == BLE_SAMPLE_QUEUE_DEPTH) {
            ring->head = (ring->head + 1) % BLE_SAMPLE_QUEUE_DEPTH;
            --ring->count;
            dropped = true;
        }
        size_t index = (ring->head + ring->count) % BLE_SAMPLE_QUEUE_DEPTH;
        memcpy(ring->storage + index * ring->item_size, sample, ring->item_size);
        ++ring->count;
    }
    xSemaphoreGive(ring->lock);
    if (dropped) ble_sample_buffer_note_dropped(1);
}

void ble_sample_buffer_publish_raw(const imu_raw_sample_t *sample, uint32_t epoch)
{ publish(BLE_BUFFER_RAW, sample, epoch); }
void ble_sample_buffer_publish_dmp(const imu_dmp_sample_t *sample, uint32_t epoch)
{ publish(BLE_BUFFER_DMP, sample, epoch); }
void ble_sample_buffer_publish_aux(const adc_sample_t *sample, uint32_t epoch)
{ publish(BLE_BUFFER_AUX, sample, epoch); }

static size_t take(unsigned kind, void *out, size_t limit, uint32_t *epoch)
{
    sample_ring_t *ring = ring_for(kind);
    if (!ring || !ring->lock || !out || !limit || !epoch) return 0;
    xSemaphoreTake(ring->lock, portMAX_DELAY);
    *epoch = ring->epoch;
    if (!ring->enabled || !ring->count) {
        xSemaphoreGive(ring->lock);
        return 0;
    }

    size_t head = ring->head;
    const uint8_t *first = ring->storage + head * ring->item_size;
    sample_prefix_t first_prefix;
    memcpy(&first_prefix, first, sizeof(first_prefix));
    memcpy(out, first, ring->item_size);
    head = (head + 1) % BLE_SAMPLE_QUEUE_DEPTH;

    if (limit > ring->count) limit = ring->count;
    size_t count = 1;
    uint32_t previous_sequence_delta = 0, previous_time_delta = 0;
    for (; count < limit; ++count) {
        const uint8_t *next = ring->storage + head * ring->item_size;
        sample_prefix_t next_prefix;
        memcpy(&next_prefix, next, sizeof(next_prefix));
        uint32_t sequence_delta = next_prefix.sequence - first_prefix.sequence;
        uint32_t time_delta = next_prefix.timestamp_ms - first_prefix.timestamp_ms;
        if (sequence_delta <= previous_sequence_delta || sequence_delta > UINT8_MAX ||
            time_delta < previous_time_delta || time_delta > UINT16_MAX) break;
        if (kind == BLE_BUFFER_RAW) {
            const imu_raw_sample_t *first_raw = (const imu_raw_sample_t *)first;
            const imu_raw_sample_t *next_raw = (const imu_raw_sample_t *)next;
            if (first_raw->accel_range_code != next_raw->accel_range_code ||
                first_raw->gyro_range_code != next_raw->gyro_range_code ||
                first_raw->actual_rate_hz != next_raw->actual_rate_hz) break;
        }
        memcpy((uint8_t *)out + count * ring->item_size, next, ring->item_size);
        previous_sequence_delta = sequence_delta;
        previous_time_delta = time_delta;
        head = (head + 1) % BLE_SAMPLE_QUEUE_DEPTH;
    }
    ring->head = head;
    ring->count -= count;
    xSemaphoreGive(ring->lock);
    return count;
}

size_t ble_sample_buffer_take_raw(imu_raw_sample_t *out, size_t limit, uint32_t *epoch)
{ return take(BLE_BUFFER_RAW, out, limit, epoch); }
size_t ble_sample_buffer_take_dmp(imu_dmp_sample_t *out, size_t limit, uint32_t *epoch)
{ return take(BLE_BUFFER_DMP, out, limit, epoch); }
size_t ble_sample_buffer_take_aux(adc_sample_t *out, size_t limit, uint32_t *epoch)
{ return take(BLE_BUFFER_AUX, out, limit, epoch); }
