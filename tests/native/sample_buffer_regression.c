/* Host contract tests: compare observable record lists, epochs and counters.
 * No production queue internals or codec implementation are included here.
 */
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ble/ble_sample_buffer.h"
#include "freertos/semphr.h"
#include "system_state.h"

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #expr); \
    abort(); \
} } while (0)
#define LENGTH(a) (sizeof(a) / sizeof((a)[0]))

/* Static host mutexes avoid SDK allocation and are destroyed at process exit. */
static pthread_mutex_t host_mutexes[3];
static unsigned mutex_count;
static atomic_uint dropped;

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    CHECK(mutex_count < LENGTH(host_mutexes));
    pthread_mutex_t *mutex = &host_mutexes[mutex_count++];
    CHECK(pthread_mutex_init(mutex, NULL) == 0);
    return mutex;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t timeout)
{
    CHECK(timeout == portMAX_DELAY);
    CHECK(pthread_mutex_lock(mutex) == 0);
    return pdTRUE;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t mutex)
{
    CHECK(pthread_mutex_unlock(mutex) == 0);
    return pdTRUE;
}

/* Compile against the real struct so a wrong production counter is a failure. */
void system_state_increment_counter(size_t offset)
{
    CHECK(offset == offsetof(system_state_t, dropped_sensor_samples));
    atomic_fetch_add_explicit(&dropped, 1, memory_order_relaxed);
}

typedef union {
    imu_raw_sample_t raw;
    imu_dmp_sample_t dmp;
    adc_sample_t aux;
} record_t;

/* Deliberately distinct signed values, flags and metadata expose torn copies. */
static record_t record(unsigned kind, uint32_t sequence, uint32_t timestamp)
{
    record_t value = {0};
    int16_t v = (int16_t)(sequence % 10000U);
    if (kind == BLE_BUFFER_RAW) {
        value.raw = (imu_raw_sample_t){
            .sequence = sequence, .timestamp_ms = timestamp,
            .accel_x = v, .accel_y = (int16_t)-v, .accel_z = (int16_t)(v + 1),
            .gyro_x = (int16_t)(v + 2), .gyro_y = (int16_t)(-v - 3),
            .gyro_z = (int16_t)(v + 4), .mag_x = (int16_t)(-v - 5),
            .mag_y = (int16_t)(v + 6), .mag_z = (int16_t)(-v - 7),
            .temperature = (int16_t)(v + 8),
            .actual_rate_hz = 100,
            .flags = (uint8_t)(sequence % 128U),
            .accel_range_code = 2, .gyro_range_code = 1,
        };
    } else if (kind == BLE_BUFFER_DMP) {
        value.dmp = (imu_dmp_sample_t){
            .sequence = sequence, .timestamp_ms = timestamp,
            .q0_q30 = INT32_MAX - v, .q1_q30 = INT32_MIN + v,
            .q2_q30 = 12345678 + v, .q3_q30 = -23456789 - v,
            .accuracy = (uint16_t)(sequence % 65536U),
            .flags = (uint8_t)(sequence % 32U),
        };
    } else {
        CHECK(kind == BLE_BUFFER_AUX);
        value.aux.sequence = sequence;
        value.aux.timestamp_ms = timestamp;
        for (size_t i = 0; i < AUX_PACKET_CHANNEL_COUNT; ++i)
            value.aux.values[i] = (int16_t)((i % 2 ? -v : v) + (int)i);
    }
    return value;
}

static void equal(unsigned kind, record_t got, record_t expected)
{
#define FIELD(member, name) CHECK(got.member.name == expected.member.name)
    if (kind == BLE_BUFFER_RAW) {
        FIELD(raw, sequence); FIELD(raw, timestamp_ms);
        FIELD(raw, accel_x); FIELD(raw, accel_y); FIELD(raw, accel_z);
        FIELD(raw, gyro_x); FIELD(raw, gyro_y); FIELD(raw, gyro_z);
        FIELD(raw, mag_x); FIELD(raw, mag_y); FIELD(raw, mag_z);
        FIELD(raw, temperature); FIELD(raw, actual_rate_hz); FIELD(raw, flags);
        FIELD(raw, accel_range_code); FIELD(raw, gyro_range_code);
    } else if (kind == BLE_BUFFER_DMP) {
        FIELD(dmp, sequence); FIELD(dmp, timestamp_ms);
        FIELD(dmp, q0_q30); FIELD(dmp, q1_q30); FIELD(dmp, q2_q30); FIELD(dmp, q3_q30);
        FIELD(dmp, accuracy); FIELD(dmp, flags);
    } else {
        FIELD(aux, sequence); FIELD(aux, timestamp_ms);
        for (size_t i = 0; i < AUX_PACKET_CHANNEL_COUNT; ++i)
            CHECK(got.aux.values[i] == expected.aux.values[i]);
    }
#undef FIELD
}

static void publish(unsigned kind, const record_t *value, uint32_t epoch)
{
    if (kind == BLE_BUFFER_RAW) ble_sample_buffer_publish_raw(value ? &value->raw : NULL, epoch);
    else if (kind == BLE_BUFFER_DMP) ble_sample_buffer_publish_dmp(value ? &value->dmp : NULL, epoch);
    else ble_sample_buffer_publish_aux(value ? &value->aux : NULL, epoch);
}

/* Real typed output arrays: a union array would have the wrong AUX stride. */
static size_t take(unsigned kind, record_t *out, size_t limit, uint32_t *epoch)
{
    union {
        imu_raw_sample_t raw[64];
        imu_dmp_sample_t dmp[64];
        adc_sample_t aux[64];
    } buffer;
    CHECK(limit <= 64);
    memset(&buffer, 0xa5, sizeof(buffer));
    size_t n;
    size_t item_size;
    if (kind == BLE_BUFFER_RAW) {
        n = ble_sample_buffer_take_raw(buffer.raw, limit, epoch);
        item_size = sizeof(buffer.raw[0]);
        CHECK(n <= limit);
        for (size_t i = 0; i < n; ++i) out[i].raw = buffer.raw[i];
    } else if (kind == BLE_BUFFER_DMP) {
        n = ble_sample_buffer_take_dmp(buffer.dmp, limit, epoch);
        item_size = sizeof(buffer.dmp[0]);
        CHECK(n <= limit);
        for (size_t i = 0; i < n; ++i) out[i].dmp = buffer.dmp[i];
    } else {
        n = ble_sample_buffer_take_aux(buffer.aux, limit, epoch);
        item_size = sizeof(buffer.aux[0]);
        CHECK(n <= limit);
        for (size_t i = 0; i < n; ++i) out[i].aux = buffer.aux[i];
    }
    /* A batch may not overwrite any slot after its reported count. */
    const unsigned char *bytes = (const unsigned char *)&buffer;
    for (size_t i = n * item_size; i < sizeof(buffer); ++i) CHECK(bytes[i] == 0xa5);
    return n;
}

static uint32_t fresh(unsigned kind)
{
    ble_sample_buffer_set_enabled(kind, true);
    ble_sample_buffer_invalidate(kind);
    return ble_sample_buffer_epoch(kind);
}

static void test_init_and_discard(void)
{
    CHECK(ble_sample_buffer_init() == ESP_OK);
    CHECK(mutex_count == 3);
    CHECK(BLE_SAMPLE_QUEUE_DEPTH == 64);
    for (unsigned kind = 1; kind <= 3; ++kind) {
        ble_sample_buffer_info_t info = ble_sample_buffer_info(kind);
        CHECK(!info.enabled && info.count == 0 && info.oldest_timestamp_ms == 0);
        record_t value = record(kind, 7, 1234), out[64];
        uint32_t epoch = info.epoch;
        publish(kind, &value, epoch);
        CHECK(take(kind, out, 64, &epoch) == 0);
        epoch = fresh(kind);
        publish(kind, &value, epoch);
        CHECK(ble_sample_buffer_init() == ESP_OK);
        CHECK(mutex_count == 3);
        CHECK(ble_sample_buffer_epoch(kind) == epoch);
        ble_sample_buffer_set_enabled(kind, true); /* Repeated subscribe is harmless. */
        CHECK(ble_sample_buffer_epoch(kind) == epoch);
        CHECK(take(kind, out, 1, &epoch) == 1);
        equal(kind, out[0], value);
        publish(kind, &value, epoch);
        ble_sample_buffer_set_enabled(kind, false);
        epoch = ble_sample_buffer_epoch(kind);
        publish(kind, &value, epoch); /* Even the current epoch cannot publish disabled. */
        CHECK(take(kind, out, 64, &epoch) == 0);
        CHECK(ble_sample_buffer_info(kind).count == 0);
    }
    CHECK(atomic_load(&dropped) == 0); /* Intentional discards are not overflow. */
    puts("PASS: idempotent init/subscribe and disabled/unsubscribed discard");
}

static void test_overflow_and_partial(void)
{
    for (unsigned kind = 1; kind <= 3; ++kind) {
        uint32_t epoch = fresh(kind), returned_epoch = 0;
        unsigned before = atomic_load(&dropped);
        record_t out[64];
        for (uint32_t i = 0; i < 64; ++i) {
            record_t value = record(kind, i, 1000 + 2 * i);
            publish(kind, &value, epoch);
        }
        CHECK(atomic_load(&dropped) == before);
        for (uint32_t i = 64; i < 81; ++i) {
            record_t value = record(kind, i, 1000 + 2 * i);
            publish(kind, &value, epoch);
        }
        CHECK(atomic_load(&dropped) == before + 17);
        ble_sample_buffer_info_t info = ble_sample_buffer_info(kind);
        CHECK(info.count == 64 && info.oldest_timestamp_ms == 1034 && info.epoch == epoch);
        CHECK(take(kind, out, 7, &returned_epoch) == 7);
        CHECK(returned_epoch == epoch);
        for (uint32_t i = 0; i < 7; ++i) equal(kind, out[i], record(kind, i + 17, 1034 + 2 * i));
        CHECK(ble_sample_buffer_info(kind).count == 57);
        /* Refill after a partial drain to force physical ring wrap without loss. */
        for (uint32_t i = 81; i < 88; ++i) {
            record_t value = record(kind, i, 1000 + 2 * i);
            publish(kind, &value, epoch);
        }
        CHECK(take(kind, out, 64, &returned_epoch) == 64);
        for (uint32_t i = 0; i < 64; ++i) equal(kind, out[i], record(kind, i + 24, 1048 + 2 * i));
        CHECK(take(kind, out, 64, &returned_epoch) == 0);
        CHECK(ble_sample_buffer_info(kind).oldest_timestamp_ms == 0);
        CHECK(atomic_load(&dropped) == before + 17);
    }
    unsigned before = atomic_load(&dropped);
    ble_sample_buffer_note_dropped(0);
    CHECK(atomic_load(&dropped) == before);
    ble_sample_buffer_note_dropped(5);
    CHECK(atomic_load(&dropped) == before + 5);
    puts("PASS: 64-record capacity, oldest drop/counter, chronological partial take and ring wrap");
}

/* Explicit expected batch lengths, independent of the compatibility algorithm. */
static void batches(unsigned kind, const record_t *input, size_t count,
                    const size_t *lengths, size_t batch_count)
{
    uint32_t epoch = fresh(kind), returned_epoch = 0;
    unsigned before = atomic_load(&dropped);
    for (size_t i = 0; i < count; ++i) publish(kind, &input[i], epoch);
    size_t position = 0;
    for (size_t b = 0; b < batch_count; ++b) {
        record_t out[64];
        CHECK(take(kind, out, 64, &returned_epoch) == lengths[b]);
        CHECK(returned_epoch == epoch);
        for (size_t i = 0; i < lengths[b]; ++i) equal(kind, out[i], input[position + i]);
        position += lengths[b];
        CHECK(ble_sample_buffer_info(kind).count == count - position);
        if (position < count) {
            uint32_t time = kind == 1 ? input[position].raw.timestamp_ms :
                            kind == 2 ? input[position].dmp.timestamp_ms : input[position].aux.timestamp_ms;
            CHECK(ble_sample_buffer_info(kind).oldest_timestamp_ms == time);
        }
    }
    CHECK(position == count);
    CHECK(atomic_load(&dropped) == before);
}

static void test_batch_boundaries(void)
{
    static const struct {
        uint32_t seq[4], time[4];
        size_t sizes[3], batch_count;
    } cases[] = {
        /* Simultaneous 32-bit sequence/time wrap, with equal timestamps. */
        {{UINT32_MAX-1, UINT32_MAX, 0, 1}, {UINT32_MAX-2, UINT32_MAX, 0, 0}, {4}, 1},
        /* Inclusive wire delta boundaries; the next sample must survive a split. */
        {{100, 355, 356, 357}, {200, 201, 202, 203}, {2, 2}, 2},
        {{100, 101, 102, 103}, {200, 65735, 65736, 65737}, {2, 2}, 2},
        /* Duplicate/backward sequence and backward time must start a new batch. */
        {{10, 11, 11, 12}, {100, 101, 102, 103}, {2, 2}, 2},
        {{10, 11, 9, 10}, {100, 101, 102, 103}, {2, 2}, 2},
        {{10, 11, 12, 13}, {100, 101, 100, 101}, {2, 2}, 2},
        /* Delta too large immediately after the first record. */
        {{10, 266, 267, 268}, {100, 101, 102, 103}, {1, 3}, 2},
        {{10, 11, 12, 13}, {100, 65636, 65637, 65638}, {1, 3}, 2},
    };
    for (unsigned kind = 1; kind <= 3; ++kind) {
        for (size_t c = 0; c < LENGTH(cases); ++c) {
            record_t input[4];
            for (size_t i = 0; i < LENGTH(input); ++i)
                input[i] = record(kind, cases[c].seq[i], cases[c].time[i]);
            batches(kind, input, LENGTH(input), cases[c].sizes, cases[c].batch_count);
        }
    }
    record_t raw[4];
    for (unsigned i = 0; i < LENGTH(raw); ++i) raw[i] = record(1, i + 1, i + 100);
    raw[1].raw.accel_range_code = 3;
    raw[2].raw.accel_range_code = 3;
    raw[2].raw.gyro_range_code = 2;
    raw[3].raw.accel_range_code = 3;
    raw[3].raw.gyro_range_code = 2;
    const size_t range_batches[] = {1, 1, 2};
    batches(1, raw, LENGTH(raw), range_batches, LENGTH(range_batches));
    /* Measured rate is header metadata. A rolling rate-window update must
     * split before codec validation, never discard a mixed-rate batch. */
    for (unsigned i = 0; i < LENGTH(raw); ++i) {
        raw[i] = record(1, i + 1, i + 100);
        raw[i].raw.actual_rate_hz = i < 2 ? 44 : 45;
    }
    const size_t rate_batches[] = {2, 2};
    batches(1, raw, LENGTH(raw), rate_batches, LENGTH(rate_batches));
    puts("PASS: u32 sequence/time wrap, measured-rate/range metadata and delta splits preserve next record");
}

/* Pause an acquisition after it snapshots the epoch, then finish it after a
 * lifecycle change. This models the producer/reset race without timing sleeps. */
typedef struct {
    unsigned kind;
    atomic_bool captured, release;
} inflight_t;

static void *late_producer(void *arg)
{
    inflight_t *flight = arg;
    uint32_t epoch = ble_sample_buffer_epoch(flight->kind);
    record_t stale = record(flight->kind, 999, 999);
    atomic_store(&flight->captured, true);
    while (!atomic_load(&flight->release)) sched_yield();
    publish(flight->kind, &stale, epoch);
    return NULL;
}

static void test_epochs(void)
{
    for (unsigned kind = 1; kind <= 3; ++kind) {
        for (unsigned action = 0; action < 3; ++action) {
            uint32_t old_epoch = fresh(kind), taken_epoch;
            unsigned before = atomic_load(&dropped);
            record_t old = record(kind, 1, 100), out[64];
            publish(kind, &old, old_epoch);
            CHECK(take(kind, out, 1, &taken_epoch) == 1);
            CHECK(taken_epoch == old_epoch);
            publish(kind, &old, old_epoch); /* Pending records must also be cleared. */
            inflight_t flight = {.kind = kind};
            pthread_t thread;
            CHECK(pthread_create(&thread, NULL, late_producer, &flight) == 0);
            while (!atomic_load(&flight.captured)) sched_yield();
            if (action == 0) { /* Unsubscribe/resubscribe. */
                ble_sample_buffer_set_enabled(kind, false);
                ble_sample_buffer_set_enabled(kind, true);
            } else if (action == 1) { /* Disconnect/reconnect. */
                ble_sample_buffer_disable_all();
                ble_sample_buffer_set_enabled(kind, true);
            } else { /* Sensor/FIFO reset while subscription remains enabled. */
                ble_sample_buffer_invalidate(kind);
            }
            uint32_t new_epoch = ble_sample_buffer_epoch(kind);
            CHECK(new_epoch != taken_epoch);
            CHECK(ble_sample_buffer_info(kind).enabled);
            CHECK(ble_sample_buffer_info(kind).count == 0);
            record_t current = record(kind, 2, 200);
            publish(kind, &current, new_epoch);
            atomic_store(&flight.release, true);
            CHECK(pthread_join(thread, NULL) == 0);
            CHECK(take(kind, out, 64, &taken_epoch) == 1);
            CHECK(taken_epoch == new_epoch);
            equal(kind, out[0], current);
            CHECK(atomic_load(&dropped) == before);
        }
    }
    puts("PASS: old in-flight epochs rejected after unsubscribe, reconnect and reset");
}

static void test_independence_and_guards(void)
{
    uint32_t epochs[3];
    for (unsigned kind = 1; kind <= 3; ++kind) {
        epochs[kind-1] = fresh(kind);
        record_t value = record(kind, 50 * kind, 1000 * kind);
        publish(kind, &value, epochs[kind-1]);
    }
    ble_sample_buffer_invalidate(BLE_BUFFER_RAW);
    CHECK(ble_sample_buffer_info(BLE_BUFFER_RAW).count == 0);
    ble_sample_buffer_set_enabled(BLE_BUFFER_RAW, false);
    for (unsigned kind = 2; kind <= 3; ++kind) {
        record_t out[64];
        uint32_t epoch = 0;
        CHECK(ble_sample_buffer_epoch(kind) == epochs[kind-1]);
        CHECK(take(kind, out, 64, &epoch) == 1);
        equal(kind, out[0], record(kind, 50 * kind, 1000 * kind));
    }
    for (unsigned kind = 1; kind <= 3; ++kind) {
        uint32_t epoch = fresh(kind), out_epoch = UINT32_MAX;
        record_t value = record(kind, 42, 99), out[64];
        publish(kind, &value, epoch);
        publish(kind, NULL, epoch);
        CHECK(take(kind, out, 0, &out_epoch) == 0);
        CHECK(out_epoch == UINT32_MAX);
        CHECK(take(kind, out, 64, NULL) == 0);
        if (kind == 1) CHECK(ble_sample_buffer_take_raw(NULL, 1, &out_epoch) == 0);
        else if (kind == 2) CHECK(ble_sample_buffer_take_dmp(NULL, 1, &out_epoch) == 0);
        else CHECK(ble_sample_buffer_take_aux(NULL, 1, &out_epoch) == 0);
        CHECK(ble_sample_buffer_info(kind).count == 1);
        CHECK(take(kind, out, 64, &out_epoch) == 1);
        equal(kind, out[0], value);
    }
    ble_sample_buffer_set_enabled(0, true);
    ble_sample_buffer_invalidate(4);
    CHECK(ble_sample_buffer_info(0).count == 0);
    CHECK(!ble_sample_buffer_info(UINT_MAX).enabled);
    ble_sample_buffer_disable_all();
    for (unsigned kind = 1; kind <= 3; ++kind) CHECK(!ble_sample_buffer_info(kind).enabled);
    puts("PASS: stream isolation and null/zero/invalid-kind guards");
}

#define CONCURRENT_RECORDS 30000U
static atomic_bool start;
typedef struct {
    unsigned kind;
    uint32_t epoch;
    atomic_bool done;
    size_t consumed;
} stream_t;

static void *producer(void *arg)
{
    stream_t *stream = arg;
    while (!atomic_load(&start)) sched_yield();
    for (uint32_t i = 1; i <= CONCURRENT_RECORDS; ++i) {
        record_t value = record(stream->kind, i, UINT32_MAX - 1000U + i * 3U);
        publish(stream->kind, &value, stream->epoch);
        /* Overwriting the source exposes accidental storage of caller pointers. */
        memset(&value, 0x5a, sizeof(value));
        if (i % 71U == 0) sched_yield();
    }
    atomic_store(&stream->done, true);
    return NULL;
}

static void *consumer(void *arg)
{
    stream_t *stream = arg;
    uint32_t last = 0;
    while (!atomic_load(&start)) sched_yield();
    for (;;) {
        record_t out[64];
        uint32_t epoch = 0;
        size_t n = take(stream->kind, out, 13, &epoch);
        CHECK(epoch == stream->epoch);
        for (size_t i = 0; i < n; ++i) {
            uint32_t seq = stream->kind == 1 ? out[i].raw.sequence :
                           stream->kind == 2 ? out[i].dmp.sequence : out[i].aux.sequence;
            CHECK(seq > last && seq <= CONCURRENT_RECORDS);
            equal(stream->kind, out[i], record(stream->kind, seq, UINT32_MAX - 1000U + seq * 3U));
            last = seq;
            ++stream->consumed;
        }
        CHECK(ble_sample_buffer_info(stream->kind).count <= 64);
        if (atomic_load(&stream->done) && ble_sample_buffer_info(stream->kind).count == 0) break;
        if (!n) sched_yield();
    }
    CHECK(last == CONCURRENT_RECORDS);
    return NULL;
}

static void test_concurrency(void)
{
    unsigned before = atomic_load(&dropped);
    stream_t streams[3] = {0};
    pthread_t producers[3], consumers[3];
    atomic_store(&start, false);
    for (unsigned i = 0; i < 3; ++i) {
        streams[i].kind = i + 1;
        streams[i].epoch = fresh(i + 1);
        CHECK(pthread_create(&producers[i], NULL, producer, &streams[i]) == 0);
        CHECK(pthread_create(&consumers[i], NULL, consumer, &streams[i]) == 0);
    }
    atomic_store(&start, true);
    size_t consumed = 0;
    for (unsigned i = 0; i < 3; ++i) {
        CHECK(pthread_join(producers[i], NULL) == 0);
        CHECK(pthread_join(consumers[i], NULL) == 0);
        consumed += streams[i].consumed;
        CHECK(ble_sample_buffer_info(i + 1).count == 0);
    }
    unsigned lost = atomic_load(&dropped) - before;
    CHECK(consumed + lost == 3U * CONCURRENT_RECORDS);
    printf("PASS: concurrent 3 producers/3 consumers: %zu intact ordered records + %u counted drops = %u\n",
           consumed, lost, 3U * CONCURRENT_RECORDS);
}

int main(void)
{
    test_init_and_discard();
    test_overflow_and_partial();
    test_batch_boundaries();
    test_epochs();
    test_independence_and_guards();
    test_concurrency();
    for (unsigned i = 0; i < mutex_count; ++i) CHECK(pthread_mutex_destroy(&host_mutexes[i]) == 0);
    puts("PASS: all sample buffer regression groups");
    return 0;
}
