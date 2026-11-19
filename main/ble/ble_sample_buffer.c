#include "ble_sample_buffer.h"

#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "system_state.h"

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
static imu_dmp_sample_t s_dmp[BLE_SAMPLE_QUEUE_DEPTH];
static adc_sample_t s_aux[BLE_SAMPLE_QUEUE_DEPTH];
static sample_ring_t s_rings[3] = {
    {.storage = (uint8_t *)s_raw, .item_size = sizeof(s_raw[0])},
    {.storage = (uint8_t *)s_dmp, .item_size = sizeof(s_dmp[0])},
    {.storage = (uint8_t *)s_aux, .item_size = sizeof(s_aux[0])},
};

static sample_ring_t *ring_for(unsigned kind)
{
    return kind >= BLE_BUFFER_RAW && kind <= BLE_BUFFER_AUX ? &s_rings[kind - 1] : NULL;
}

esp_err_t ble_sample_buffer_init(void)
{
    /* Application initialization is serialized, before acquisition starts. */
    for (unsigned i = 0; i < 3; ++i) {
        if (s_rings[i].lock) continue;
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
    return ble_sample_buffer_info(kind).epoch;
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

static bool compatible(unsigned kind, const void *first, const void *next,
                       uint32_t previous_sequence_delta, uint32_t previous_time_delta)
{
    uint32_t first_sequence, next_sequence, first_time, next_time;
    memcpy(&first_sequence, first, sizeof(uint32_t));
    memcpy(&next_sequence, next, sizeof(uint32_t));
    memcpy(&first_time, (const uint8_t *)first + 4, sizeof(uint32_t));
    memcpy(&next_time, (const uint8_t *)next + 4, sizeof(uint32_t));
    uint32_t sequence_delta = next_sequence - first_sequence;
    uint32_t time_delta = next_time - first_time;
    if (sequence_delta <= previous_sequence_delta || sequence_delta > UINT8_MAX ||
        time_delta < previous_time_delta || time_delta > UINT16_MAX) return false;
    if (kind == BLE_BUFFER_RAW) {
        const imu_raw_sample_t *a = first, *b = next;
        if (a->accel_range_code != b->accel_range_code ||
            a->gyro_range_code != b->gyro_range_code ||
            a->actual_rate_hz != b->actual_rate_hz) return false;
    }
    return true;
}

static size_t take(unsigned kind, void *out, size_t limit, uint32_t *epoch)
{
    sample_ring_t *ring = ring_for(kind);
    if (!ring || !ring->lock || !out || !limit || !epoch) return 0;
    xSemaphoreTake(ring->lock, portMAX_DELAY);
    *epoch = ring->epoch;
    size_t count = 0;
    uint32_t previous_sequence_delta = 0, previous_time_delta = 0;
    while (ring->enabled && count < limit && ring->count) {
        const void *next = ring->storage + ring->head * ring->item_size;
        if (count && !compatible(kind, out, next, previous_sequence_delta, previous_time_delta)) break;
        uint32_t sequence, first_sequence, timestamp, first_time;
        memcpy(&sequence, next, 4);
        memcpy(&timestamp, (const uint8_t *)next + 4, 4);
        memcpy((uint8_t *)out + count * ring->item_size, next, ring->item_size);
        memcpy(&first_sequence, out, 4);
        memcpy(&first_time, (const uint8_t *)out + 4, 4);
        previous_sequence_delta = sequence - first_sequence;
        previous_time_delta = timestamp - first_time;
        ring->head = (ring->head + 1) % BLE_SAMPLE_QUEUE_DEPTH;
        --ring->count;
        ++count;
    }
    xSemaphoreGive(ring->lock);
    return count;
}

size_t ble_sample_buffer_take_raw(imu_raw_sample_t *out, size_t limit, uint32_t *epoch)
{ return take(BLE_BUFFER_RAW, out, limit, epoch); }
size_t ble_sample_buffer_take_dmp(imu_dmp_sample_t *out, size_t limit, uint32_t *epoch)
{ return take(BLE_BUFFER_DMP, out, limit, epoch); }
size_t ble_sample_buffer_take_aux(adc_sample_t *out, size_t limit, uint32_t *epoch)
{ return take(BLE_BUFFER_AUX, out, limit, epoch); }
