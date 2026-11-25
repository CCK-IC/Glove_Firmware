/* Deterministic host scheduling; two task iterations per race, no sleeps. */
#include <assert.h>
#include <setjmp.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include "ble/ble_sample_buffer.h"
#include "ble/gatt_server.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#define BLE_HS_ENOTCONN 7
#define BLE_HS_ENOMEM 6
#define BLE_HS_CONN_HANDLE_NONE UINT16_MAX
#define ESP_LOGW(...) ((void)0)
struct ble_npl_event { int unused; };
struct os_mempool { unsigned mp_num_free; };
struct os_mbuf_pool { struct os_mempool *omp_pool; };
struct os_mbuf {
    uint8_t data[512];
    size_t length;
    struct os_mbuf_pool *om_omp;
};
struct native_sem { bool mutex; unsigned value; };
static struct native_sem sems[4];
static unsigned sem_count;
static struct native_sem connection_mutex = {.mutex = true};
static SemaphoreHandle_t s_connection_lock = &connection_mutex;
static uint16_t s_connection_handle = 1;
static atomic_bool s_notifications_enabled = true;
static atomic_uint s_connection_generation = 1;
static system_state_t state;
static bool subscribed[5];
static uint32_t other_epochs[2];
static int64_t now_us;
static bool queued, auto_host;
static unsigned accepted, attempts, iterations, fail_attempts;
static struct os_mempool mbuf_mem;
static unsigned mbufs_freed;
static unsigned trace[64];
static uint8_t first_attempt[512];
static size_t first_length;
static jmp_buf task_exit;
enum { NORMAL, MTU_RACE, EXPIRY_RACE, EPOCH_RACE, RECONNECT_RACE, RETRY_EXPIRY,
       DISCONNECT_EXPIRY_RACE, SNAPSHOT_RECONNECT_RACE, MBUF_RESERVE };
static unsigned scenario;
static void run_host(void);
static void publish_raw(unsigned count);
static void maybe_request_connection_update(void);
static void disconnect_link(void);
static bool disconnect_injected;
static void check_pairing_timeout(void) {}
static void vTaskDelay(TickType_t ticks);
static struct os_mbuf *ble_hs_mbuf_from_flat(const void *data, uint16_t length);
static int os_mbuf_free_chain(struct os_mbuf *om);
static int ble_gatts_notify_custom(uint16_t conn, uint16_t handle, struct os_mbuf *om);
static void *nimble_port_get_dflt_eventq(void) { return NULL; }
static void ble_npl_eventq_put(void *queue, struct ble_npl_event *event)
{ (void)queue; (void)event; assert(!queued); queued = true; }
bool gatt_stream_subscribed(unsigned kind) { return subscribed[kind - 1]; }
uint16_t gatt_stream_handle(unsigned kind) { return 10 + kind; }
uint32_t gatt_stream_epoch(unsigned kind)
{
    uint32_t epoch = kind <= 3 ? ble_sample_buffer_epoch(kind) : other_epochs[kind - 4];
    /* Disconnect just after the pending frame's old epoch was read, before
     * timeout handling can record a loss. */
    if (scenario == DISCONNECT_EXPIRY_RACE && iterations == 1 &&
        !disconnect_injected && !connection_mutex.value) {
        disconnect_injected = true;
        disconnect_link();
    }
    return epoch;
}

#include "sender_under_test.inc"

int64_t esp_timer_get_time(void) { return now_us; }
uint32_t esp_get_free_heap_size(void) { return 100000; }
void system_state_increment_counter(size_t offset)
{ ++*(uint32_t *)((uint8_t *)&state + offset); }
void system_state_write_field(size_t offset, const void *value, size_t size)
{ memcpy((uint8_t *)&state + offset, value, size); }
void system_state_snapshot(system_state_t *out)
{ *out = state; }
static void disconnect_link(void)
{
    xSemaphoreTake(s_connection_lock, portMAX_DELAY);
    s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
    atomic_fetch_add(&s_connection_generation, 1);
    ble_sample_buffer_disable_all();
    memset(subscribed, 0, sizeof(subscribed));
    state.ble_connected = false;
    state.mtu = 23;
    xSemaphoreGive(s_connection_lock);
}
static void maybe_request_connection_update(void)
{
    if (scenario == SNAPSHOT_RECONNECT_RACE && iterations == 1) {
        disconnect_link();
        xSemaphoreTake(s_connection_lock, portMAX_DELAY);
        s_connection_handle = 1; /* Simulate recycling the connection handle. */
        atomic_fetch_add(&s_connection_generation, 1);
        state.ble_connected = true;
        state.mtu = 247;
        subscribed[0] = true;
        ble_sample_buffer_set_enabled(1, true);
        xSemaphoreGive(s_connection_lock);
        publish_raw(2);
    }
    /* First finish_host_job already observed not-done. Let the host finish
     * before housekeeping, reproducing an ordinary task preemption. */
    if (iterations == 1 && queued) {
        run_host();
        auto_host = true;
        if (scenario == EXPIRY_RACE) now_us = 1500000;
    }
}
SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    assert(sem_count < 3);
    sems[sem_count].mutex = true;
    return &sems[sem_count++];
}
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout)
{
    if (sem->mutex) { assert(!sem->value); sem->value = 1; return pdTRUE; }
    if (!sem->value && timeout && auto_host && queued) run_host();
    if (!sem->value) return 0;
    sem->value = 0;
    return pdTRUE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem)
{
    if (sem->mutex) { assert(sem->value); sem->value = 0; }
    else sem->value = 1;
    return pdTRUE;
}
static void run_host(void)
{ assert(queued); queued = false; notification_host_event(&s_tx_event); }
static struct os_mbuf *ble_hs_mbuf_from_flat(const void *data, uint16_t length)
{
    static struct os_mbuf_pool pool = {.omp_pool = &mbuf_mem};
    static struct os_mbuf om = {.om_omp = &pool};
    /* No sample lock may be held while calling BLE. */
    for (unsigned i = 0; i < sem_count; ++i) assert(!sems[i].value);
    assert(!connection_mutex.value);
    assert(length <= sizeof(om.data));
    memcpy(om.data, data, length); om.length = length;
    return &om;
}
static int os_mbuf_free_chain(struct os_mbuf *om)
{
    assert(om && om->om_omp->omp_pool == &mbuf_mem);
    ++mbufs_freed;
    return 0;
}
static int ble_gatts_notify_custom(uint16_t conn, uint16_t handle, struct os_mbuf *om)
{
    assert(conn == s_connection_handle && handle == 10 + s_tx_job.kind);
    assert(attempts < 64);
    if (!attempts) { first_length = om->length; memcpy(first_attempt, om->data, om->length); }
    if (fail_attempts) {
        assert(first_length == om->length && !memcmp(first_attempt, om->data, om->length));
        --fail_attempts; ++attempts; return BLE_HS_ENOMEM;
    }
    if (scenario == NORMAL && attempts && !subscribed[1])
        assert(first_length == om->length && !memcmp(first_attempt, om->data, om->length));
    ++attempts; trace[accepted++] = s_tx_job.kind;
    return 0;
}
static void publish_raw(unsigned count)
{
    uint32_t epoch = ble_sample_buffer_epoch(1);
    for (unsigned i = 0; i < count; ++i) {
        imu_raw_sample_t sample = {.sequence = i + 1, .timestamp_ms = 900 + i,
            .actual_rate_hz = 45, .flags = 1};
        ble_sample_buffer_publish_raw(&sample, epoch);
    }
}
static void vTaskDelay(TickType_t ticks)
{
    (void)ticks;
    if (++iterations >= 2 || scenario == NORMAL) longjmp(task_exit, 1);
    if (scenario == RETRY_EXPIRY || scenario == DISCONNECT_EXPIRY_RACE) {
        now_us += 500000;
        return;
    }
    if (scenario == SNAPSHOT_RECONNECT_RACE) return;
    if (scenario == MBUF_RESERVE) { mbuf_mem.mp_num_free = ATT_MBUF_RESERVE; return; }
    assert(queued);
    if (scenario == MTU_RACE) state.mtu = 247;
    if (scenario == EXPIRY_RACE) now_us = 1499000;
    if (scenario == EPOCH_RACE || scenario == RECONNECT_RACE) {
        ble_sample_buffer_set_enabled(1, false);
        ble_sample_buffer_set_enabled(1, true);
        if (scenario == RECONNECT_RACE) atomic_fetch_add(&s_connection_generation, 2);
    }
}
static void reset(unsigned test)
{
    ble_sample_buffer_disable_all();
    memset(s_pending, 0, sizeof(s_pending));
    memset(subscribed, 0, sizeof(subscribed));
    memset(&state, 0, sizeof(state));
    state.ble_connected = true; state.mtu = 128; state.status_period_ms = 1000;
    now_us = 1000000; scenario = test;
    auto_host = test == NORMAL || test == RETRY_EXPIRY ||
        test == DISCONNECT_EXPIRY_RACE || test == MBUF_RESERVE;
    accepted = attempts = iterations = fail_attempts = mbufs_freed = 0;
    mbuf_mem.mp_num_free = test == MBUF_RESERVE ? ATT_MBUF_RESERVE - 1 : 100;
    disconnect_injected = false;
    s_connection_handle = 1;
    queued = s_tx_pending = false;
    atomic_store(&s_tx_done, false);
    atomic_store(&s_connection_generation, 1);
    s_tx_ack = &sems[3]; sems[3] = (struct native_sem){0};
    subscribed[0] = true; ble_sample_buffer_set_enabled(1, true);
    publish_raw(2);
}
static void run_task(void)
{ if (setjmp(task_exit) == 0) notification_task(NULL); }
static unsigned failures;
#define EXPECT(condition, description) do { \
    bool ok = (condition); printf("%s: %s\n", ok ? "PASS" : "FAIL", description); \
    if (!ok) ++failures; \
} while (0)
int main(void)
{
    (void)s_connection_lock;
    assert(ble_sample_buffer_init() == ESP_OK);
    reset(NORMAL); fail_attempts = 2; run_task();
    EXPECT(attempts == 3 && accepted == 1 && !state.dropped_sensor_samples &&
        !state.dropped_notifications, "transient enqueue retries preserve exact bytes, no drops");
    reset(EPOCH_RACE); run_task();
    EXPECT(accepted == 0 && !s_tx_pending, "unsubscribe/resubscribe rejects queued old-epoch job");
    reset(RECONNECT_RACE); run_task();
    EXPECT(accepted == 0 && !s_tx_pending, "reconnect rejects queued old-generation job");
    reset(MTU_RACE); run_task();
    printf("  MTU race: accepted=%u (expected 1)\n", accepted);
    EXPECT(accepted == 1, "MTU change during outstanding job must not duplicate complete batch");
    reset(EXPIRY_RACE); run_task();
    printf("  Expiry race: accepted=%u sampleDrops=%u notificationDrops=%u\n",
        accepted, state.dropped_sensor_samples, state.dropped_notifications);
    EXPECT(accepted == 1 && !state.dropped_sensor_samples && !state.dropped_notifications,
        "completed final enqueue before deadline must not be counted as expired");
    reset(RETRY_EXPIRY); fail_attempts = 20; run_task();
    EXPECT(accepted == 0 && attempts == 6 && state.dropped_sensor_samples == 2 &&
        state.dropped_notifications == 1 && !s_pending[0].length,
        "persistent enqueue failure expires once at 500ms and counts its exact samples");
    reset(DISCONNECT_EXPIRY_RACE); fail_attempts = 20; run_task();
    EXPECT(disconnect_injected && !state.ble_connected && !state.dropped_sensor_samples &&
        !state.dropped_notifications && !s_pending[0].length,
        "disconnect between epoch check and expiration cancels loss accounting");
    reset(SNAPSHOT_RECONNECT_RACE); subscribed[0] = false;
    ble_sample_buffer_set_enabled(1, false);
    run_task();
    EXPECT(ble_sample_buffer_info(1).count == 2 && !attempts && !s_tx_pending,
        "sender cannot drain a reconnected peer using old session metadata");
    reset(MBUF_RESERVE); run_task();
    EXPECT(mbufs_freed == TX_BUDGET && attempts == 1 && accepted == 1 &&
        !state.dropped_sensor_samples && !state.dropped_notifications,
        "ATT pool headroom defers stream fragments until capacity is available");
    reset(NORMAL); state.mtu = 23;
    for (unsigned kind = 2; kind <= 3; ++kind) {
        subscribed[kind - 1] = true; ble_sample_buffer_set_enabled(kind, true);
        for (unsigned i = 0; i < 8; ++i) {
            uint32_t epoch = ble_sample_buffer_epoch(kind);
            imu_dmp_sample_t dmp = {.sequence = i + 1, .timestamp_ms = 900 + i, .flags = 1};
            adc_sample_t aux = {.sequence = i + 1, .timestamp_ms = 900 + i};
            if (kind == 2) ble_sample_buffer_publish_dmp(&dmp, epoch);
            else ble_sample_buffer_publish_aux(&aux, epoch);
        }
    }
    subscribed[3] = subscribed[4] = true;
    run_task();
    EXPECT(accepted == 6 && trace[0] == 1 && trace[1] == 2 && trace[2] == 3 &&
        trace[3] == 4 && trace[4] == 5 && trace[5] == 1,
        "MTU23 fragments rotate raw/DMP/AUX/status/battery within six-job budget");
    return failures ? 1 : 0;
}
