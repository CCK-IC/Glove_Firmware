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
#include "imu/imu_manager.h"
#include "adc/adc_manager.h"
#include "ble_packets.h"
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

    system_state_t state;
    system_state_snapshot(&state);
    state.advertising = true;
    SYSTEM_STATE_SET(advertising, state.advertising);
}

static int handle_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    system_state_t state;
    system_state_snapshot(&state);

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "connect status=%d handle=%u",
                event->connect.status,
                event->connect.conn_handle);

        if (event->connect.status != 0) {
            system_state_snapshot(&state);
            system_state_set_ble_connected(false);
            state.advertising = false;
            SYSTEM_STATE_SET(advertising, state.advertising);

            start_advertising();
            return 0;
        }

        xSemaphoreTake(s_connection_lock, portMAX_DELAY);
        s_connection_handle = event->connect.conn_handle;
        s_connection_update_due_us = 0;
        s_connection_update_attempted = false;
        xSemaphoreGive(s_connection_lock);

        state.conn_handle = event->connect.conn_handle;
        SYSTEM_STATE_SET(conn_handle, state.conn_handle);
        system_state_set_ble_connected(true);
        state.advertising = false;
        SYSTEM_STATE_SET(advertising, state.advertising);
        state.encrypted = false;
        SYSTEM_STATE_SET(encrypted, state.encrypted);
        state.bonded = false;
        SYSTEM_STATE_SET(bonded, state.bonded);
        state.mtu = ble_att_mtu(event->connect.conn_handle);
        SYSTEM_STATE_SET(mtu, state.mtu);
        if (state.mtu < 23) {
            state.mtu = 23;
            SYSTEM_STATE_SET(mtu, state.mtu);
        }

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
        atomic_fetch_add(&s_connection_generation, 1);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
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

        xSemaphoreTake(s_connection_lock, portMAX_DELAY);
        s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
        s_connection_update_due_us = 0;
        s_connection_update_attempted = false;
        xSemaphoreGive(s_connection_lock);
        gatt_clear_subscriptions();

        system_state_set_ble_connected(false);
        state.encrypted = false;
        SYSTEM_STATE_SET(encrypted, state.encrypted);
        state.bonded = false;
        SYSTEM_STATE_SET(bonded, state.bonded);
        state.conn_handle = UINT16_MAX;
        SYSTEM_STATE_SET(conn_handle, state.conn_handle);
        state.mtu = 23;
        SYSTEM_STATE_SET(mtu, state.mtu);
        state.slow_mode = false;
        SYSTEM_STATE_SET(slow_mode, state.slow_mode);
        state.requested_imu_rate_hz = APP_IMU_NORMAL_RATE_HZ;
        SYSTEM_STATE_SET(requested_imu_rate_hz, state.requested_imu_rate_hz);

        start_advertising();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG, "connection update status=%d", event->conn_update.status);
        log_connection("updated", event->conn_update.conn_handle);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        state.advertising = false;
        SYSTEM_STATE_SET(advertising, state.advertising);

        start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        gatt_set_subscription(event->subscribe.attr_handle,
                              event->subscribe.cur_notify);
        return 0;

    case BLE_GAP_EVENT_MTU:
        state.mtu = event->mtu.value;
        SYSTEM_STATE_SET(mtu, state.mtu);

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
            state.encrypted = desc.sec_state.encrypted;
            SYSTEM_STATE_SET(encrypted, state.encrypted);
            state.bonded = desc.sec_state.bonded;
            SYSTEM_STATE_SET(bonded, state.bonded);

            if (event->enc_change.status == 0) {
                xSemaphoreTake(s_connection_lock, portMAX_DELAY);
                s_pairing_deadline_us = 0;
                state.pairing = false;
                SYSTEM_STATE_SET(pairing, state.pairing);
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
    s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
    s_connection_update_due_us = 0;
    s_connection_update_attempted = false;
    s_pairing_deadline_us = 0;
    xSemaphoreGive(s_connection_lock);
    system_state_set_ble_connected(false);
    SYSTEM_STATE_SET(conn_handle, UINT16_MAX);
    SYSTEM_STATE_SET(mtu, 23);
    SYSTEM_STATE_SET(advertising, false);
    SYSTEM_STATE_SET(encrypted, false);
    SYSTEM_STATE_SET(bonded, false);
    SYSTEM_STATE_SET(pairing, false);
    gatt_clear_subscriptions();
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

static int enqueue_notification(uint16_t handle, const void *data, uint16_t len)
{
    xSemaphoreTake(s_connection_lock, portMAX_DELAY);
    uint16_t connection_handle = s_connection_handle;
    xSemaphoreGive(s_connection_lock);
    if (connection_handle == BLE_HS_CONN_HANDLE_NONE) return BLE_HS_ENOTCONN;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) return BLE_HS_ENOMEM;
    return ble_gatts_notify_custom(connection_handle, handle, om);
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

static void notification_task(void *arg)
{
    (void)arg;

    TickType_t wake = xTaskGetTickCount();
    uint64_t rate_window_us = esp_timer_get_time();
    uint64_t status_due_us = rate_window_us;
#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY || CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
    uint64_t next_imu_due_us = 0;
    uint32_t last_imu_rate_hz = 0;
#endif
    unsigned connection_generation = 0;

    uint32_t sent_in_window = 0;
#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY
    uint32_t last_imu_sequence = UINT32_MAX;
#endif
#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
    uint32_t last_raw_sequence = UINT32_MAX;
#endif
#if CONFIG_AUX_ADC_ENABLED
    uint32_t last_adc_sequence = UINT32_MAX;
#endif
    uint8_t last_battery = UINT8_MAX;

    while (true) {
        system_state_t state;
        system_state_snapshot(&state);

        maybe_request_connection_update();
        check_pairing_timeout();

        uint64_t now_us = esp_timer_get_time();
        unsigned current_generation = atomic_load(&s_connection_generation);
        if (current_generation != connection_generation) {
            connection_generation = current_generation;
            last_battery = UINT8_MAX;
            status_due_us = now_us;
#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY || CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
            next_imu_due_us = now_us;
#endif
#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY
            last_imu_sequence = UINT32_MAX;
#endif
#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
            last_raw_sequence = UINT32_MAX;
#endif
#if CONFIG_AUX_ADC_ENABLED
            last_adc_sequence = UINT32_MAX;
#endif
        }

        uint32_t imu_rate_hz =
            state.requested_imu_rate_hz ? state.requested_imu_rate_hz : 1;
        bool streaming = atomic_load(&s_notifications_enabled) && state.ble_connected;
#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY || CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
        if (imu_rate_hz != last_imu_rate_hz) {
            next_imu_due_us = now_us;
            last_imu_rate_hz = imu_rate_hz;
        }
        bool imu_due = now_us >= next_imu_due_us;
        if (imu_due) {
            uint64_t interval_us = 1000000ULL / imu_rate_hz;
            /* Skip missed intervals while retaining phase across rounded task ticks. */
            next_imu_due_us += ((now_us - next_imu_due_us) / interval_us + 1) * interval_us;
        }
#endif
        uint32_t poll_rate_hz = 10; /* Remain responsive without polling idle streams. */
        if (state.ble_connected && state.status_period_ms > 0 &&
            (gatt_status_subscribed() || gatt_battery_subscribed())) {
            uint32_t status_rate_hz = (1000U + state.status_period_ms - 1) /
                                      state.status_period_ms;
            if (status_rate_hz > poll_rate_hz) poll_rate_hz = status_rate_hz;
        }
        if (streaming) {
            if (gatt_dmp_subscribed() || gatt_imu_raw_subscribed())
                poll_rate_hz = imu_rate_hz > poll_rate_hz ? imu_rate_hz : poll_rate_hz;
#if CONFIG_AUX_ADC_ENABLED
            if (gatt_aux_subscribed() && state.requested_adc_rate_hz > poll_rate_hz)
                poll_rate_hz = state.requested_adc_rate_hz;
#endif
        }

        TickType_t period =
            pdMS_TO_TICKS(1000U / poll_rate_hz);

        if (period == 0) {
            period = 1;
        }
#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY
        if (streaming && imu_due && gatt_dmp_subscribed()) {
            imu_dmp_sample_t sample;
            if (imu_manager_latest_dmp(&sample)) {
                bool is_new = sample.sequence != last_imu_sequence;
#if CONFIG_IMU_NOTIFY_DUPLICATE_SAMPLES
                is_new = true;
#endif
                if (is_new) {
                    uint8_t packet[IMU_PACKET_Q30_SIZE];
                    size_t length = ble_encode_imu_for_mtu(
                        packet, sizeof(packet), state.mtu, &sample);
                    if (length == 0 || enqueue_notification(gatt_dmp_handle, packet, length) != 0) {
                        SYSTEM_STATE_INCREMENT(dropped_notifications);
                    } else {
                        ++sent_in_window;
                    }
                    /* Consume samples once, including recoverable enqueue failures. */
                    last_imu_sequence = sample.sequence;
                }
            }
        }
#endif
#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
        if (streaming && imu_due &&
            gatt_imu_raw_subscribed()) {

            imu_raw_sample_t raw_sample;

            if (imu_manager_latest_raw(&raw_sample) &&
                raw_sample.sequence != last_raw_sequence) {
                uint8_t packet[IMU_RAW_FULL_V1_SIZE];

                size_t length = ble_encode_imu_raw_for_mtu(
                    packet,
                    sizeof(packet),
                    state.mtu,
                    &raw_sample);

                if (length == 0) {
                    SYSTEM_STATE_INCREMENT(dropped_notifications);

                    last_raw_sequence = raw_sample.sequence;
                } else {
                    int rc = enqueue_notification(
                        gatt_imu_raw_handle,
                        packet,
                        (uint16_t)length);

                    if (rc != 0) {
                        SYSTEM_STATE_INCREMENT(dropped_notifications);
                        static unsigned raw_notify_failures;
                        if (++raw_notify_failures <= 5 || raw_notify_failures % 100 == 0) {
                            ESP_LOGW(TAG, "raw notification enqueue failure rc=%d count=%u",
                                     rc, raw_notify_failures);
                        }
                    } else {
                        sent_in_window++;
                    }

                    /*
                     * Consume each raw sample once. Notification errors are
                     * recoverable and do not terminate the connection.
                     */
                    last_raw_sequence = raw_sample.sequence;
                }
            }
        }
#endif
#if CONFIG_AUX_ADC_ENABLED
        if (streaming &&
            gatt_aux_subscribed()) {
            adc_sample_t adc_sample;
            if ( adc_manager_latest(&adc_sample) && adc_sample.sequence != last_adc_sequence) {
                uint8_t packet[AUX_PACKET_V1_SIZE];
                size_t length = ble_encode_aux_for_mtu(
                    packet,
                    sizeof(packet),
                    state.mtu,
                    &adc_sample);

                if (length == 0) {
                    SYSTEM_STATE_INCREMENT(dropped_notifications);

                    last_adc_sequence = adc_sample.sequence;
                } else {
                    int rc = enqueue_notification(
                        gatt_aux_handle,
                        packet,
                        (uint16_t)length);

                    if (rc != 0) {
                        SYSTEM_STATE_INCREMENT(dropped_notifications);
                        static unsigned aux_notify_failures;
                        if (++aux_notify_failures <= 5 || aux_notify_failures % 100 == 0)
                            ESP_LOGW(TAG, "AUX notification enqueue failure rc=%d count=%u",
                                     rc, aux_notify_failures);
                    }
                    /*
                     * Consume each AUX sample once. Notification errors are
                     * recoverable and do not terminate the connection.
                     */
                    last_adc_sequence = adc_sample.sequence;
                }
            }
        }
#endif

        now_us = esp_timer_get_time();

        if (now_us - rate_window_us >= 1000000ULL) {
            SYSTEM_STATE_SET(actual_imu_notify_rate_hz,
                             sent_in_window * 1000000ULL / (now_us - rate_window_us));

            sent_in_window = 0;
            rate_window_us = now_us;
        }

        system_state_snapshot(&state);

        if (now_us >= status_due_us) {
            status_due_us =
                now_us + state.status_period_ms * 1000ULL;

            if (state.ble_connected &&
                gatt_status_subscribed()) {
                uint8_t packet[STATUS_PACKET_FULL_SIZE];

                size_t length = ble_encode_status_for_mtu(
                    packet,
                    sizeof(packet),
                    state.mtu);

                if (length != 0) {
                    int rc = enqueue_notification(
                        gatt_status_handle,
                        packet,
                        (uint16_t)length);

                    if (rc != 0) {
                        SYSTEM_STATE_INCREMENT(dropped_notifications);
                    }
                }
            }

            if (state.ble_connected &&
                gatt_battery_subscribed() &&
                state.battery_percent != last_battery) {
                int rc = enqueue_notification(
                    gatt_battery_handle,
                    &state.battery_percent,
                    sizeof(state.battery_percent));

                if (rc == 0) {
                    last_battery = state.battery_percent;
                } else {
                    SYSTEM_STATE_INCREMENT(dropped_notifications);
                }
            }
        }

        vTaskDelayUntil(&wake, period);
    }
}

esp_err_t ble_manager_init(void)
{
    s_connection_lock = xSemaphoreCreateMutex();
    if (!s_connection_lock) return ESP_ERR_NO_MEM;

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) return err;

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_server_register_cb;
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
}
