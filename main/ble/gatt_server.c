#include "gatt_server.h"
#include "ble_uuid.h"
#include "ble_packets.h"
#include "app_events.h"
#include "app_config.h"
#include "system_state.h"
#include "imu/imu_manager.h"
#include "adc/adc_manager.h"
#include "power/power_manager.h"

#include <stdatomic.h>
#include <string.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

const ble_uuid128_t UUID_IMU_SERVICE = XIAO_UUID128(0x01);
const ble_uuid128_t UUID_IMU_QUAT = XIAO_UUID128(0x02);
const ble_uuid128_t UUID_IMU_RAW = XIAO_UUID128(0x99);
const ble_uuid128_t UUID_AUX_SERVICE = XIAO_UUID128(0x10);
const ble_uuid128_t UUID_AUX_DATA = XIAO_UUID128(0x11);
const ble_uuid128_t UUID_STATUS_SERVICE = XIAO_UUID128(0x20);
const ble_uuid128_t UUID_STATUS_DATA = XIAO_UUID128(0x21);
const ble_uuid128_t UUID_CONTROL = XIAO_UUID128(0x22);

uint16_t gatt_dmp_handle;
uint16_t gatt_imu_raw_handle;
uint16_t gatt_aux_handle;
uint16_t gatt_status_handle;
uint16_t gatt_control_handle;
uint16_t gatt_battery_handle;

static atomic_bool s_dmp_notifications_enabled;
static atomic_bool s_raw_notifications_enabled;
static atomic_bool s_aux_notifications_enabled;
static atomic_bool s_status_notifications_enabled;
static atomic_bool s_battery_notifications_enabled;
static uint8_t s_control_reply[4];

/* Wire command values are part of the client protocol. */
typedef enum {
    CONTROL_SET_IMU_RATE = 0x01,
    CONTROL_SET_STATUS_PERIOD = 0x02,
    CONTROL_START_PAIRING = 0x03,
    CONTROL_SET_SLOW_MODE = 0x04,
    CONTROL_POWER_OFF = 0x05,
    CONTROL_RESET_IMU_FIFO = 0x06,
} control_command_t;

static uint16_t effective_mtu(uint16_t conn_handle)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return 23;
    }

    uint16_t mtu = ble_att_mtu(conn_handle);
    return mtu >= 23 ? mtu : 23;
}

static int append_read_response(struct os_mbuf *om, const void *data, size_t len)
{
    return os_mbuf_append(om, data, len) == 0
        ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int copy_control_write(struct os_mbuf *om, void *dst, uint16_t max,
                      uint16_t *len)
{
    uint16_t n = OS_MBUF_PKTLEN(om);
    if (n > max) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    return ble_hs_mbuf_to_flat(om, dst, max, len) == 0
        ? 0 : BLE_ATT_ERR_UNLIKELY;
}

static int handle_gatt_access(uint16_t conn, uint16_t attr,
                     struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY
        if (attr == gatt_dmp_handle) {
            imu_dmp_sample_t sample = {0};
            uint8_t packet[IMU_PACKET_Q30_SIZE];
            if (!imu_manager_latest_dmp(&sample)) {
                return BLE_ATT_ERR_UNLIKELY;
            }
            size_t len = ble_encode_imu_for_mtu(
                packet,
                sizeof(packet),
                effective_mtu(conn),
                &sample);
            if (len == 0) {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            return append_read_response(ctxt->om, packet, len);
        }
#endif
        if (attr == gatt_imu_raw_handle) {
#if CONFIG_IMU_RAW_ENABLED || CONFIG_IMU_RAW_DUMMY
            imu_raw_sample_t sample = {0};
            uint8_t packet[IMU_RAW_FULL_V1_SIZE];
            if (!imu_manager_latest_raw(&sample)) {
                return BLE_ATT_ERR_UNLIKELY;
            }
            size_t len = ble_encode_imu_raw_for_mtu(
                packet,
                sizeof(packet),
                effective_mtu(conn),
                &sample);
            if (len == 0) {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            return append_read_response(ctxt->om, packet, len);
#else
            return BLE_ATT_ERR_READ_NOT_PERMITTED;
#endif
        }
        if (attr == gatt_aux_handle) {
            adc_sample_t sample;
            if (!adc_manager_latest(&sample)) return BLE_ATT_ERR_UNLIKELY;
            uint8_t data[AUX_PACKET_V1_SIZE];
            ble_encode_aux(data, sample.timestamp_ms, sample.sequence, sample.values);
            return append_read_response(ctxt->om, data, sizeof(data));
        }
        if (attr == gatt_status_handle) {
            uint8_t packet[STATUS_PACKET_FULL_SIZE];
            size_t len = ble_encode_status_for_mtu(
                packet,
                sizeof(packet),
                effective_mtu(conn));
            if (len == 0) {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            return append_read_response(ctxt->om, packet, len);
        }
        if (attr == gatt_control_handle) {
            return append_read_response(ctxt->om, s_control_reply,
                          sizeof(s_control_reply));
        }
        if (attr == gatt_battery_handle) {
            system_state_t state;
            system_state_snapshot(&state);
            return append_read_response(ctxt->om, &state.battery_percent, 1);
        }

        static const char manufacturer[] = "Seeed Studio / application";
        static const char model[] = APP_NAME;
        if (ble_uuid_u16(ctxt->chr->uuid) == 0x2A29)
            return append_read_response(ctxt->om, manufacturer, sizeof(manufacturer) - 1);
        if (ble_uuid_u16(ctxt->chr->uuid) == 0x2A24)
            return append_read_response(ctxt->om, model, sizeof(model) - 1);

        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR &&
        attr == gatt_control_handle) {
        uint8_t data[8];
        uint16_t len = 0;
        int rc = copy_control_write(ctxt->om, data, sizeof(data), &len);
        if (rc) return rc;
        if (len < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

        uint16_t value = len >= 3 ? (uint16_t)data[1] |
                                   ((uint16_t)data[2] << 8) : 0;
        s_control_reply[0] = data[0];
        s_control_reply[1] = 0;

        system_state_t state;
        system_state_snapshot(&state);

        switch (data[0]) {
        case CONTROL_SET_IMU_RATE:
            if (len != 3 || value < 1 || value > 200)
                return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
            state.requested_imu_rate_hz = value;
            SYSTEM_STATE_SET(requested_imu_rate_hz, state.requested_imu_rate_hz);

            break;
        case CONTROL_SET_STATUS_PERIOD:
            if (len != 3 || value < 250)
                return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
            state.status_period_ms = value;
            SYSTEM_STATE_SET(status_period_ms, state.status_period_ms);

            break;
        case CONTROL_START_PAIRING:
            return app_event_publish(APP_EVENT_BUTTON_HOLD, 0) == ESP_OK
                ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case CONTROL_SET_SLOW_MODE:
            if (len != 2 || data[1] > 1)
                return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
            state.slow_mode = data[1];
            SYSTEM_STATE_SET(slow_mode, state.slow_mode);
            state.requested_imu_rate_hz = data[1] ?
                APP_IMU_SLOW_RATE_HZ : APP_IMU_NORMAL_RATE_HZ;
            SYSTEM_STATE_SET(requested_imu_rate_hz, state.requested_imu_rate_hz);
            ESP_LOGI("BLE_CTRL", "slow_mode=%u, rate=%luHz",
                     state.slow_mode,
                     (unsigned long)state.requested_imu_rate_hz);

            break;
        case CONTROL_POWER_OFF:
            if (len != 3 || data[1] != 0xA5 || data[2] != 0x5A)
                return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
            return app_event_publish(APP_EVENT_POWER_OFF, POWER_OFF_REASON_COMMAND) == ESP_OK
                ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case CONTROL_RESET_IMU_FIFO:
            return app_event_publish(APP_EVENT_IMU_REINIT, 0) == ESP_OK
                ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        default:
            return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        }
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180F),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0x2A19),
                .access_cb = handle_gatt_access,
                .val_handle = &gatt_battery_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            }, {0}
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0x2A29),
                .access_cb = handle_gatt_access,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                .uuid = BLE_UUID16_DECLARE(0x2A24),
                .access_cb = handle_gatt_access,
                .flags = BLE_GATT_CHR_F_READ,
            }, {0}
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_IMU_SERVICE.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
#if CONFIG_IMU_DMP_ENABLED || CONFIG_IMU_DMP_DUMMY
            {
                .uuid = &UUID_IMU_QUAT.u,
                .access_cb = handle_gatt_access,
                .val_handle = &gatt_dmp_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
#endif
            {
                .uuid = &UUID_IMU_RAW.u,
                .access_cb = handle_gatt_access,
                .val_handle = &gatt_imu_raw_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            }, {0}
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_AUX_SERVICE.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &UUID_AUX_DATA.u,
                .access_cb = handle_gatt_access,
                .val_handle = &gatt_aux_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            }, {0}
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_STATUS_SERVICE.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &UUID_STATUS_DATA.u,
                .access_cb = handle_gatt_access,
                .val_handle = &gatt_status_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            }, {
                .uuid = &UUID_CONTROL.u,
                .access_cb = handle_gatt_access,
                .val_handle = &gatt_control_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_NOTIFY,
            }, {0}
        }
    },
    {0}
};

int gatt_server_init(void)
{
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(services);
    if (rc) return rc;
    return ble_gatts_add_svcs(services);
}

void gatt_server_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    (void)ctxt;
    (void)arg;
}

void gatt_set_subscription(uint16_t attr, bool enabled)
{
    if (attr == gatt_dmp_handle) {
        atomic_store(&s_dmp_notifications_enabled, enabled);
        SYSTEM_STATE_SET(imu_dmp_subscribed, enabled);
    } else if (attr == gatt_imu_raw_handle) {
        atomic_store(&s_raw_notifications_enabled, enabled);
        SYSTEM_STATE_SET(imu_raw_subscribed, enabled);
    } else if (attr == gatt_aux_handle) {
        atomic_store(&s_aux_notifications_enabled, enabled);
        SYSTEM_STATE_SET(aux_subscribed, enabled);
    } else if (attr == gatt_status_handle) {
        atomic_store(&s_status_notifications_enabled, enabled);
    } else if (attr == gatt_battery_handle) {
        atomic_store(&s_battery_notifications_enabled, enabled);
    }
}

void gatt_clear_subscriptions(void)
{
    atomic_store(&s_dmp_notifications_enabled, false);
    atomic_store(&s_raw_notifications_enabled, false);
    atomic_store(&s_aux_notifications_enabled, false);
    atomic_store(&s_status_notifications_enabled, false);
    atomic_store(&s_battery_notifications_enabled, false);
    SYSTEM_STATE_SET(imu_dmp_subscribed, false);
    SYSTEM_STATE_SET(imu_raw_subscribed, false);
    SYSTEM_STATE_SET(aux_subscribed, false);
}

bool gatt_dmp_subscribed(void) { return atomic_load(&s_dmp_notifications_enabled); }
bool gatt_imu_raw_subscribed(void) { return atomic_load(&s_raw_notifications_enabled); }
bool gatt_aux_subscribed(void) { return atomic_load(&s_aux_notifications_enabled); }
bool gatt_status_subscribed(void) { return atomic_load(&s_status_notifications_enabled); }
bool gatt_battery_subscribed(void) { return atomic_load(&s_battery_notifications_enabled); }
