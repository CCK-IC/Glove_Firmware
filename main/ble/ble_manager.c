#include "ble_manager.h"

#include <string.h>
#include <stdio.h>
#include <stdatomic.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

#include "app_config.h"
#include "app_events.h"
#include "system_state.h"
#include "ble_batch_packets.h"
#include "ble_sample_buffer.h"
#include "ble_uuid.h"
#include "gatt_server.h"

void ble_store_config_init(void);

static const char *TAG = "ble";
static uint8_t s_address_type;
/* Host callbacks and the notification task share this connection metadata. */
static SemaphoreHandle_t s_connection_lock;
static uint16_t s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
static atomic_bool s_notifications_enabled = true;
static atomic_uint s_connection_generation;
static int64_t s_pairing_deadline_us;
static int64_t s_connection_update_due_us;
static bool s_connection_update_attempted;

enum { BATCH_FLUSH_MS = 80, BATCH_RETRY_MS = 500, TX_BUDGET = 6,
       ATT_MBUF_RESERVE = 4 };

typedef struct {
    size_t length;
    size_t samples;
    unsigned kind;
    uint16_t handle;
    uint16_t mtu;
    uint16_t frame_id;
    unsigned fragment;
    uint32_t epoch;
    unsigned generation;
    int64_t created_us;
} pending_frame_t;

static const struct {
    uint8_t *data;
    size_t capacity;
} s_pending_payloads[GATT_STREAM_COUNT] = {
    [BLE_BATCH_KIND_RAW - 1] = {(uint8_t[BLE_BATCH_RAW_MAX_SIZE]){0}, BLE_BATCH_RAW_MAX_SIZE},
#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY
    [BLE_BATCH_KIND_DMP - 1] = {(uint8_t[BLE_BATCH_DMP_MAX_SIZE]){0}, BLE_BATCH_DMP_MAX_SIZE},
#endif
    [BLE_BATCH_KIND_AUX - 1] = {(uint8_t[BLE_BATCH_AUX_MAX_SIZE]){0}, BLE_BATCH_AUX_MAX_SIZE},
    [BLE_BATCH_KIND_STATUS - 1] = {(uint8_t[BLE_BATCH_STATUS_SIZE]){0}, BLE_BATCH_STATUS_SIZE},
    [GATT_STREAM_BATTERY - 1] = {(uint8_t[1]){0}, 1},
};
static pending_frame_t s_pending[GATT_STREAM_COUNT];
static uint16_t s_frame_ids[GATT_STREAM_COUNT];
static struct ble_npl_event s_tx_event;
static SemaphoreHandle_t s_tx_ack;
static bool s_tx_pending; /* Owned only by the notification task. */
static atomic_bool s_tx_done;
static atomic_int s_tx_result;
static struct {
    uint8_t data[BLE_BATCH_MAX_ENCODED_SIZE];
    uint16_t length;
    uint16_t handle;
    uint16_t frame_id;
    unsigned kind;
    unsigned fragment;
    unsigned generation;
    uint32_t epoch;
    int64_t deadline_us;
} s_tx_job;

static const char *hci_reason_name(int reason)
{
    switch (reason & 0xff) {
    case 0x08:
        return "connection timeout";
    case 0x13:
        return "remote user terminated";
    case 0x16:
        return "local host terminated";
    case 0x3b:
        return "unacceptable connection parameters";
    default:
        return "unclassified";
    }
}

static void log_connection(
    const char *prefix,
    uint16_t connection_handle)
{
#if CONFIG_BLE_LOG_GAP_DETAILS
    struct ble_gap_conn_desc desc;
    int rc = ble_gap_conn_find(connection_handle, &desc);

    if (rc != 0) {
        ESP_LOGW(TAG, "%s: conn_find rc=%d handle=%u",
                 prefix, rc, connection_handle);
        return;
    }

    ESP_LOGI(TAG,
             "%s: handle=%u interval=%u(1.25ms) latency=%u "
             "timeout=%u(10ms) encrypted=%u authenticated=%u bonded=%u",
             prefix,
             desc.conn_handle,
             desc.conn_itvl,
             desc.conn_latency,
             desc.supervision_timeout,
             desc.sec_state.encrypted,
             desc.sec_state.authenticated,
             desc.sec_state.bonded);
#else
    (void)prefix;
    (void)connection_handle;
#endif
}

static void maybe_request_connection_update(void)
{
#if CONFIG_BLE_REQUEST_CONNECTION_UPDATE
    int64_t now_us = esp_timer_get_time();
    xSemaphoreTake(s_connection_lock, portMAX_DELAY);
    uint16_t connection_handle = s_connection_handle;
    if (connection_handle == BLE_HS_CONN_HANDLE_NONE ||
        s_connection_update_attempted ||
        s_connection_update_due_us == 0 ||
        now_us < s_connection_update_due_us) {
        xSemaphoreGive(s_connection_lock);
        return;
    }
    s_connection_update_attempted = true;
    xSemaphoreGive(s_connection_lock);

    struct ble_gap_upd_params params = {
        .itvl_min = CONFIG_BLE_CONN_INTERVAL_MIN_UNITS,
        .itvl_max = CONFIG_BLE_CONN_INTERVAL_MAX_UNITS,
        .latency = CONFIG_BLE_CONN_LATENCY,
        .supervision_timeout =
            CONFIG_BLE_CONN_SUPERVISION_TIMEOUT_UNITS,
        .min_ce_len = 0,
        .max_ce_len = 0,
    };

    int rc = ble_gap_update_params(connection_handle, &params);
    ESP_LOGI(TAG, "connection update request handle=%u rc=%d timeout=%u",
             connection_handle, rc, params.supervision_timeout);
    if (rc != 0) {
        /*
         * Connection-parameter rejection is recoverable and must not cause
         * a disconnect.
         */
        ESP_LOGW(TAG,
                 "connection update request rc=%d; retaining peer defaults",
                 rc);
    }
#endif
}

static int handle_gap_event(struct ble_gap_event *event, void *arg);

static void start_advertising(void)
{
    if (ble_gap_adv_active()) return;

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc) {
        ESP_LOGE(TAG, "adv fields rc=%d", rc);
        return;
    }

    struct ble_hs_adv_fields scan = {0};
    scan.uuids128 = (ble_uuid128_t *)&UUID_IMU_SERVICE;
    scan.num_uuids128 = 1;
    scan.uuids128_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&scan);
    if (rc) {
        ESP_LOGE(TAG, "scan response rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params p = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    rc = ble_gap_adv_start(s_address_type, NULL, BLE_HS_FOREVER,
                           &p, handle_gap_event, NULL);
    if (rc) {
        ESP_LOGE(TAG, "adv start rc=%d", rc);
        return;
    }

    SYSTEM_STATE_SET(advertising, true);
}

/* Both link teardown paths invalidate outstanding sender jobs and reads. */
static void clear_connection_locked(void)
{
    s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
    s_connection_update_due_us = 0;
    s_connection_update_attempted = false;
    atomic_fetch_add(&s_connection_generation, 1);
    gatt_clear_subscriptions();
    SYSTEM_STATE_SET(conn_handle, UINT16_MAX);
    SYSTEM_STATE_SET(mtu, 23);
    SYSTEM_STATE_SET(encrypted, false);
    SYSTEM_STATE_SET(bonded, false);
}

static int handle_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT: {
        ESP_LOGI(TAG, "connect status=%d handle=%u",
                event->connect.status,
                event->connect.conn_handle);

        if (event->connect.status != 0) {
            system_state_set_ble_connected(false);
            SYSTEM_STATE_SET(advertising, false);

            start_advertising();
            return 0;
        }

        uint16_t mtu = ble_att_mtu(event->connect.conn_handle);
        xSemaphoreTake(s_connection_lock, portMAX_DELAY);
        s_connection_handle = event->connect.conn_handle;
        s_connection_update_due_us = 0;
        s_connection_update_attempted = false;
        SYSTEM_STATE_SET(conn_handle, s_connection_handle);
        SYSTEM_STATE_SET(mtu, mtu < 23 ? 23 : mtu);
        SYSTEM_STATE_SET(advertising, false);
        SYSTEM_STATE_SET(encrypted, false);
        SYSTEM_STATE_SET(bonded, false);
        system_state_set_ble_connected(true);
        atomic_fetch_add(&s_connection_generation, 1);
        xSemaphoreGive(s_connection_lock);

        log_connection("connected", event->connect.conn_handle);

    #if CONFIG_BLE_INITIATE_SECURITY_ON_CONNECT
        {
            int rc = ble_gap_security_initiate(event->connect.conn_handle);
            if (rc != 0) {
                ESP_LOGW(TAG,
                        "security initiation rejected rc=%d; "
                        "connection retained",
                        rc);
            }
        }
    #endif

        /* Request a longer supervision timeout only when the peer's timeout
         * is too short for service discovery on macOS. */
    #if CONFIG_BLE_REQUEST_CONNECTION_UPDATE
        struct ble_gap_conn_desc connected_desc;
        if (ble_gap_conn_find(event->connect.conn_handle, &connected_desc) == 0 &&
            connected_desc.supervision_timeout < 200) {
            xSemaphoreTake(s_connection_lock, portMAX_DELAY);
            s_connection_update_due_us =
                esp_timer_get_time() +
                CONFIG_BLE_CONNECTION_UPDATE_DELAY_MS * 1000LL;
            xSemaphoreGive(s_connection_lock);
        }
    #endif
        return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT:
        xSemaphoreTake(s_connection_lock, portMAX_DELAY);
        clear_connection_locked();
        SYSTEM_STATE_SET(slow_mode, false);
        SYSTEM_STATE_SET(requested_imu_rate_hz, APP_IMU_NORMAL_RATE_HZ);
#if CONFIG_AUX_ADC_ENABLED
        SYSTEM_STATE_SET(requested_adc_rate_hz, CONFIG_ADC_RATE_HZ);
#endif
        system_state_set_ble_connected(false);
        xSemaphoreGive(s_connection_lock);

        ESP_LOGW(TAG,
                "disconnect reason=0x%04x low-byte=0x%02x (%s)",
                event->disconnect.reason,
                event->disconnect.reason & 0xff,
                hci_reason_name(event->disconnect.reason));

    #if CONFIG_BLE_LOG_GAP_DETAILS
        ESP_LOGI(TAG,
                "last connection: interval=%u latency=%u timeout=%u "
                "encrypted=%u authenticated=%u bonded=%u",
                event->disconnect.conn.conn_itvl,
                event->disconnect.conn.conn_latency,
                event->disconnect.conn.supervision_timeout,
                event->disconnect.conn.sec_state.encrypted,
                event->disconnect.conn.sec_state.authenticated,
                event->disconnect.conn.sec_state.bonded);
    #endif

        start_advertising();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG, "connection update status=%d", event->conn_update.status);
        log_connection("updated", event->conn_update.conn_handle);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        SYSTEM_STATE_SET(advertising, false);
        start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        xSemaphoreTake(s_connection_lock, portMAX_DELAY);
        /* Matching the handle also accepts bonded CCC restore events. */
        if (s_connection_handle != BLE_HS_CONN_HANDLE_NONE &&
            event->subscribe.conn_handle == s_connection_handle) {
            gatt_set_subscription(event->subscribe.attr_handle,
                event->subscribe.cur_notify && atomic_load(&s_notifications_enabled));
        }
        xSemaphoreGive(s_connection_lock);
        return 0;

    case BLE_GAP_EVENT_MTU:
        xSemaphoreTake(s_connection_lock, portMAX_DELAY);
        if (s_connection_handle != BLE_HS_CONN_HANDLE_NONE &&
            event->mtu.conn_handle == s_connection_handle) {
            SYSTEM_STATE_SET(mtu, event->mtu.value);
        }
        xSemaphoreGive(s_connection_lock);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE: {
        ESP_LOGI(TAG,
                "encryption change status=%d handle=%u",
                event->enc_change.status,
                event->enc_change.conn_handle);

        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(
            event->enc_change.conn_handle, &desc);

        if (rc == 0) {
            SYSTEM_STATE_SET(encrypted, desc.sec_state.encrypted);
            SYSTEM_STATE_SET(bonded, desc.sec_state.bonded);

            if (event->enc_change.status == 0) {
                xSemaphoreTake(s_connection_lock, portMAX_DELAY);
                s_pairing_deadline_us = 0;
                SYSTEM_STATE_SET(pairing, false);
                xSemaphoreGive(s_connection_lock);
            }

            log_connection("security state", desc.conn_handle);
        }

        /*
        * A failed optional pairing attempt is not a reason for this application
        * to terminate an otherwise usable connection.
        */
        return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        ESP_LOGW(TAG, "repeat pairing requested on handle=%u",
                event->repeat_pairing.conn_handle);

    #if CONFIG_BLE_ERASE_BONDS_ON_PAIRING
        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(
            event->repeat_pairing.conn_handle, &desc);

        if (rc == 0) {
            rc = ble_store_util_delete_peer(&desc.peer_id_addr);
            if (rc == 0) {
                return BLE_GAP_REPEAT_PAIRING_RETRY;
            }

            ESP_LOGW(TAG, "bond deletion failed rc=%d", rc);
        }
    #endif

        return BLE_GAP_REPEAT_PAIRING_IGNORE;
    }

    case BLE_GAP_EVENT_NOTIFY_TX:
        if (event->notify_tx.status != 0) {
            /* The caller already counts failed notification enqueues. */
            static unsigned notify_tx_failures;
            if (++notify_tx_failures <= 5 || notify_tx_failures % 100 == 0) {
                ESP_LOGW(TAG, "notification TX failure status=%d count=%u",
                         event->notify_tx.status, notify_tx_failures);
            }
        }
        return 0;

    default:
        return 0;
    }
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE reset reason=%d", reason);
    xSemaphoreTake(s_connection_lock, portMAX_DELAY);
    clear_connection_locked();
    s_pairing_deadline_us = 0;
    SYSTEM_STATE_SET(advertising, false);
    SYSTEM_STATE_SET(pairing, false);
    system_state_set_ble_connected(false);
    xSemaphoreGive(s_connection_lock);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc) {
        ESP_LOGE(TAG, "ensure address rc=%d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &s_address_type);
    if (rc) {
        ESP_LOGE(TAG, "infer address rc=%d", rc);
        return;
    }
    start_advertising();
}

static void host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* Called with the connection lock held, or from the NimBLE host task. */
static bool stream_is_current(unsigned kind, unsigned generation, uint32_t epoch)
{
    return atomic_load(&s_notifications_enabled) &&
        s_connection_handle != BLE_HS_CONN_HANDLE_NONE &&
        generation == atomic_load(&s_connection_generation) &&
        gatt_stream_subscribed(kind) && epoch == gatt_stream_epoch(kind);
}

/* This event executes in the NimBLE host context, serialized with connection
 * and CCC changes. Never hold an application connection lock while entering
 * NimBLE: the host callback path itself needs that lock. */
static void notification_host_event(struct ble_npl_event *event)
{
    (void)event;
    int rc = BLE_HS_ENOTCONN;
    if (stream_is_current(s_tx_job.kind, s_tx_job.generation, s_tx_job.epoch) &&
        esp_timer_get_time() < s_tx_job.deadline_us) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(s_tx_job.data, s_tx_job.length);
        if (!om) {
            rc = BLE_HS_ENOMEM;
        } else if (om->om_omp->omp_pool->mp_num_free < ATT_MBUF_RESERVE) {
            /* Check the ATT packet's actual pool; MSYS totals can hide an
             * exhausted small-block pool. Leave space for control traffic. */
            os_mbuf_free_chain(om);
            rc = BLE_HS_ENOMEM;
        } else {
            /* NimBLE consumes om on both success and failure. */
            rc = ble_gatts_notify_custom(s_connection_handle, s_tx_job.handle, om);
        }
    }
    atomic_store(&s_tx_result, rc);
    atomic_store(&s_tx_done, true);
    xSemaphoreGive(s_tx_ack);
}

static void finish_host_job(uint32_t *sent_in_window, uint8_t *last_battery)
{
    if (!s_tx_pending || !atomic_load(&s_tx_done)) return;
    int rc = atomic_load(&s_tx_result);
    s_tx_pending = false;
    pending_frame_t *frame = &s_pending[s_tx_job.kind - 1];
    if (!frame->length || frame->frame_id != s_tx_job.frame_id ||
        frame->fragment != s_tx_job.fragment ||
        frame->generation != s_tx_job.generation || frame->epoch != s_tx_job.epoch)
        return; /* A lifecycle boundary invalidated the old job. */
    if (rc != 0) {
        xSemaphoreTake(s_connection_lock, portMAX_DELAY);
        bool current = stream_is_current(frame->kind, frame->generation, frame->epoch);
        xSemaphoreGive(s_connection_lock);
        if (!current) {
            frame->length = 0;
            return;
        }
        static uint32_t failures;
        if (++failures <= 3 || failures % 100 == 0)
            ESP_LOGW(TAG, "batch enqueue retry kind=%u rc=%d count=%lu",
                     frame->kind, rc, (unsigned long)failures);
        return; /* Keep the exact same fragment for a bounded retry. */
    }
    if (frame->kind == BLE_BUFFER_RAW || frame->kind == BLE_BUFFER_DMP)
        ++*sent_in_window;
    ++frame->fragment;
    unsigned fragments = frame->kind == GATT_STREAM_BATTERY ? 1 :
        ble_batch_fragment_count(frame->length, frame->mtu);
    if (frame->fragment >= fragments) {
        if (frame->kind == GATT_STREAM_BATTERY) *last_battery = s_tx_job.data[0];
        frame->length = 0;
    }
}

static bool post_host_job(pending_frame_t *frame)
{
    if (s_tx_pending) return false;
    const uint8_t *data = s_pending_payloads[frame->kind - 1].data;
    size_t length;
    if (frame->kind == GATT_STREAM_BATTERY) {
        s_tx_job.data[0] = data[0];
        length = 1;
    } else {
        length = ble_batch_fragment(s_tx_job.data, sizeof(s_tx_job.data),
            data, frame->length, frame->mtu, frame->frame_id, frame->fragment);
    }
    if (!length) return false;
    while (xSemaphoreTake(s_tx_ack, 0) == pdTRUE) { }
    s_tx_job.length = length;
    s_tx_job.handle = frame->handle;
    s_tx_job.kind = frame->kind;
    s_tx_job.frame_id = frame->frame_id;
    s_tx_job.fragment = frame->fragment;
    s_tx_job.generation = frame->generation;
    s_tx_job.epoch = frame->epoch;
    s_tx_job.deadline_us = frame->created_us + BATCH_RETRY_MS * 1000LL;
    atomic_store(&s_tx_done, false);
    s_tx_pending = true;
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_tx_event);
    return true;
}

static void check_pairing_timeout(void)
{
    xSemaphoreTake(s_connection_lock, portMAX_DELAY);
    if (s_pairing_deadline_us != 0 &&
        esp_timer_get_time() >= s_pairing_deadline_us) {
        s_pairing_deadline_us = 0;
        SYSTEM_STATE_SET(pairing, false);
        app_event_publish(APP_EVENT_PAIRING_TIMEOUT, 0);
    }
    xSemaphoreGive(s_connection_lock);
}

static void initialize_pending(pending_frame_t *frame, unsigned kind, uint16_t mtu,
                               unsigned generation, uint32_t epoch, int64_t now_us)
{
    frame->kind = kind;
    frame->handle = gatt_stream_handle(kind);
    frame->mtu = mtu;
    frame->frame_id = ++s_frame_ids[kind - 1];
    frame->fragment = 0;
    frame->generation = generation;
    frame->epoch = epoch;
    frame->created_us = now_us;
}

static void count_stream_loss(unsigned kind, unsigned generation, uint32_t epoch,
                              size_t sample_count)
{
    xSemaphoreTake(s_connection_lock, portMAX_DELAY);
    if (stream_is_current(kind, generation, epoch)) {
        ble_sample_buffer_note_dropped(sample_count);
        SYSTEM_STATE_INCREMENT(dropped_notifications);
    }
    xSemaphoreGive(s_connection_lock);
}

static void prepare_sensor_frame(pending_frame_t *frame, unsigned kind,
                                  const system_state_t *state, unsigned generation,
                                  int64_t now_us)
{
    ble_sample_buffer_info_t info = ble_sample_buffer_info(kind);
    if (!info.enabled || !info.count) return;
    size_t limit = ble_batch_sample_limit(kind, state->mtu);
    if (!limit) return;
    uint32_t age_ms = (uint32_t)(now_us / 1000ULL) - info.oldest_timestamp_ms;
    if (age_ms >= 0x80000000U) age_ms = 0;
    if (info.count < limit && age_ms < BATCH_FLUSH_MS) return;
    union {
        imu_raw_sample_t raw[BLE_BATCH_MAX_SAMPLES];
        imu_dmp_sample_t dmp[BLE_BATCH_MAX_SAMPLES];
        adc_sample_t aux[BLE_BATCH_MAX_SAMPLES];
    } samples;
    uint32_t epoch;
    size_t count = 0;
    xSemaphoreTake(s_connection_lock, portMAX_DELAY);
    if (!stream_is_current(kind, generation, info.epoch)) {
        xSemaphoreGive(s_connection_lock);
        return;
    }
    if (kind == BLE_BUFFER_RAW)
        count = ble_sample_buffer_take_raw(samples.raw, limit, &epoch);
    else if (kind == BLE_BUFFER_DMP)
        count = ble_sample_buffer_take_dmp(samples.dmp, limit, &epoch);
    else if (kind == BLE_BUFFER_AUX)
        count = ble_sample_buffer_take_aux(samples.aux, limit, &epoch);
    xSemaphoreGive(s_connection_lock);
    if (!count) return;

    uint8_t *data = s_pending_payloads[kind - 1].data;
    size_t capacity = s_pending_payloads[kind - 1].capacity;
    size_t length;
    if (kind == BLE_BUFFER_RAW)
        length = ble_batch_encode_raw(data, capacity, samples.raw, count);
    else if (kind == BLE_BUFFER_DMP)
        length = ble_batch_encode_dmp(data, capacity, samples.dmp, count,
                                      state->actual_dmp_sample_rate_hz);
    else
        length = ble_batch_encode_aux(data, capacity, samples.aux, count,
                                      state->actual_adc_sample_rate_hz);
    if (!length) {
        count_stream_loss(kind, generation, epoch, count);
        return;
    }
    initialize_pending(frame, kind, state->mtu, generation, epoch, now_us);
    frame->length = length;
    frame->samples = count;
}

static void notification_task(void *arg)
{
    (void)arg;
    int64_t rate_window_us = esp_timer_get_time();
    int64_t status_due_us = rate_window_us;
    unsigned connection_generation = 0;
    uint32_t sent_in_window = 0;
    uint8_t last_battery = UINT8_MAX;
    unsigned cursor = 0;

    while (true) {
        finish_host_job(&sent_in_window, &last_battery);
        system_state_t state;
        xSemaphoreTake(s_connection_lock, portMAX_DELAY);
        system_state_snapshot(&state);
        unsigned generation = atomic_load(&s_connection_generation);
        bool streaming = atomic_load(&s_notifications_enabled) &&
            s_connection_handle != BLE_HS_CONN_HANDLE_NONE && state.ble_connected;
        xSemaphoreGive(s_connection_lock);
        maybe_request_connection_update();
        check_pairing_timeout();
        int64_t now_us = esp_timer_get_time();
        if (generation != connection_generation) {
            connection_generation = generation;
            memset(s_pending, 0, sizeof(s_pending));
            last_battery = UINT8_MAX;
            status_due_us = now_us;
            sent_in_window = 0;
            rate_window_us = now_us;
        }
        for (unsigned kind = 1; kind <= GATT_STREAM_COUNT; ++kind) {
            pending_frame_t *frame = &s_pending[kind - 1];
            if (!frame->length) continue;
            if (!streaming || !gatt_stream_subscribed(kind) ||
                frame->epoch != gatt_stream_epoch(kind)) {
                frame->length = 0;
            } else if (s_tx_pending && s_tx_job.kind == kind) {
                /* The host can complete after the loop's initial result check.
                 * Consume that result before expiring/reframing its batch, so
                 * an accepted final enqueue is neither duplicated nor dropped. */
                continue;
            } else if (now_us - frame->created_us >= BATCH_RETRY_MS * 1000LL) {
                count_stream_loss(kind, frame->generation, frame->epoch, frame->samples);
                frame->length = 0;
            } else if (frame->mtu != state.mtu) {
                /* A new fragment0 invalidates any partial old-MTU assembly. */
                frame->mtu = state.mtu;
                frame->frame_id = ++s_frame_ids[kind - 1];
                frame->fragment = 0;
            }
        }
        if (now_us >= status_due_us) {
            uint32_t period_ms = state.status_period_ms < 250 ? 250 : state.status_period_ms;
            status_due_us = now_us + period_ms * 1000LL;
            pending_frame_t *status = &s_pending[BLE_BATCH_KIND_STATUS - 1];
            if (streaming && gatt_stream_subscribed(BLE_BATCH_KIND_STATUS) && !status->length) {
                size_t length = ble_batch_encode_status(s_pending_payloads[BLE_BATCH_KIND_STATUS - 1].data,
                                                       BLE_BATCH_STATUS_SIZE, &state);
                if (length) {
                    initialize_pending(status, BLE_BATCH_KIND_STATUS, state.mtu, generation,
                                       gatt_stream_epoch(BLE_BATCH_KIND_STATUS), now_us);
                    status->length = length;
                    status->samples = 0;
                }
            }
            pending_frame_t *battery = &s_pending[GATT_STREAM_BATTERY - 1];
            if (streaming && gatt_stream_subscribed(GATT_STREAM_BATTERY) && !battery->length &&
                state.battery_percent != last_battery) {
                initialize_pending(battery, GATT_STREAM_BATTERY, state.mtu, generation,
                                   gatt_stream_epoch(GATT_STREAM_BATTERY), now_us);
                s_pending_payloads[GATT_STREAM_BATTERY - 1].data[0] = state.battery_percent;
                battery->length = 1;
                battery->samples = 0;
            }
        }
        /* Bound work and rotate streams so AUX/status cannot starve raw/DMP.
         * An outstanding host job is immutable even if its frame expires. */
        for (unsigned budget = 0; streaming && budget < TX_BUDGET; ++budget) {
            finish_host_job(&sent_in_window, &last_battery);
            if (s_tx_pending) break;
            pending_frame_t *selected = NULL;
            for (unsigned checked = 0; checked < GATT_STREAM_COUNT; ++checked) {
                unsigned kind = cursor + 1;
                cursor = (cursor + 1) % GATT_STREAM_COUNT;
                if (!gatt_stream_subscribed(kind)) continue;
                pending_frame_t *frame = &s_pending[kind - 1];
                if (!frame->length && kind <= BLE_BUFFER_AUX)
                    prepare_sensor_frame(frame, kind, &state, generation, esp_timer_get_time());
                if (frame->length) { selected = frame; break; }
            }
            if (!selected || !post_host_job(selected)) break;
            TickType_t wait = pdMS_TO_TICKS(10);
            xSemaphoreTake(s_tx_ack, wait ? wait : 1);
            finish_host_job(&sent_in_window, &last_battery);
        }
        now_us = esp_timer_get_time();
        if (now_us - rate_window_us >= 1000000LL) {
            SYSTEM_STATE_SET(actual_imu_notify_rate_hz,
                sent_in_window * 1000000ULL / (now_us - rate_window_us));
            sent_in_window = 0;
            rate_window_us = now_us;
        }
        TickType_t period = pdMS_TO_TICKS(streaming ? 5 : 50);
        vTaskDelay(period ? period : 1);
    }
}

esp_err_t ble_manager_init(void)
{
    s_connection_lock = xSemaphoreCreateMutex();
    if (!s_connection_lock) return ESP_ERR_NO_MEM;

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) return err;
    s_tx_ack = xSemaphoreCreateBinary();
    if (!s_tx_ack) return ESP_ERR_NO_MEM;
    ble_npl_event_init(&s_tx_event, notification_host_event, NULL);

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

#if CONFIG_BLE_SECURITY
    ble_hs_cfg.sm_bonding = CONFIG_BLE_BONDING;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;

#if CONFIG_BLE_PREFER_SECURE_CONNECTIONS
    ble_hs_cfg.sm_sc = 1;
#else
    /*
     * Compatibility profile. Legacy pairing still provides encryption and
     * bonding, but not LE Secure Connections cryptography.
     */
    ble_hs_cfg.sm_sc = 0;
#endif

    ble_hs_cfg.sm_our_key_dist =
        BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist =
        BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
#endif

#if CONFIG_BLE_DEV_STATIC_PASSKEY
    /* Insecure development-only behavior. */
    ble_sm_configure_static_passkey(CONFIG_BLE_STATIC_PASSKEY, true);
#endif

    int rc = gatt_server_init();
    if (rc) return ESP_FAIL;

    uint8_t mac[6];
    err = esp_read_mac(mac, ESP_MAC_BT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth MAC read failed: %s", esp_err_to_name(err));
        return err;
    }
    char device_name[sizeof(APP_DEVICE_NAME) + 5];
    snprintf(device_name, sizeof(device_name), "%s-%02X%02X",
             APP_DEVICE_NAME, mac[4], mac[5]);
    rc = ble_svc_gap_device_name_set(device_name);
    if (rc) return ESP_FAIL;
    ESP_LOGI(TAG, "Device name: %s", device_name);

    ble_store_config_init();
    return ESP_OK;
}

esp_err_t ble_manager_start(void)
{
    nimble_port_freertos_init(host_task);
    return xTaskCreate(notification_task, "ble_notify", 4096,
                       NULL, 7, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}

void ble_manager_start_pairing(void)
{
#if CONFIG_BLE_ERASE_BONDS_ON_PAIRING
    ble_store_clear();
#endif
    if (ble_gap_adv_active()) ble_gap_adv_stop();

    xSemaphoreTake(s_connection_lock, portMAX_DELAY);
    SYSTEM_STATE_SET(pairing, true);
    s_pairing_deadline_us = esp_timer_get_time() +
                            CONFIG_BLE_PAIRING_TIMEOUT_MS * 1000LL;
    xSemaphoreGive(s_connection_lock);
    start_advertising();
}

void ble_manager_stop_notifications(void)
{
    atomic_store(&s_notifications_enabled, false);
    /* Startup may abort before the BLE mutex has been created. */
    if (s_connection_lock) xSemaphoreTake(s_connection_lock, portMAX_DELAY);
    ble_sample_buffer_disable_all();
    if (s_connection_lock) xSemaphoreGive(s_connection_lock);
}
