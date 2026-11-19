#include "gatt_server.h"
#include "ble_batch_packets.h"
#include "ble_sample_buffer.h"
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
static atomic_uint s_status_epoch;
static atomic_uint s_battery_epoch;
/* Host callbacks are serialized. Hold one snapshot stable across ATT Read Blob. */
static uint8_t s_read_frames[4][BLE_BATCH_MAX_FRAME_SIZE];
static size_t s_read_lengths[4];

/* Wire command values are part of the client protocol. */
typedef enum {
    CONTROL_SET_IMU_RATE = 0x01,
    CONTROL_SET_STATUS_PERIOD = 0x02,
    CONTROL_START_PAIRING = 0x03,
    CONTROL_SET_SLOW_MODE = 0x04,
    CONTROL_POWER_OFF = 0x05,
    CONTROL_RESET_IMU_FIFO = 0x06,
    CONTROL_SET_ADC_RATE = 0x08,
} control_command_t;

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
    (void)conn;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        unsigned kind = attr == gatt_imu_raw_handle ? BLE_BUFFER_RAW :
                        attr == gatt_dmp_handle ? BLE_BUFFER_DMP :
                        attr == gatt_aux_handle ? BLE_BUFFER_AUX :
                        attr == gatt_status_handle ? 4 : 0;
        if (kind) {
            uint8_t *packet = s_read_frames[kind - 1];
            size_t *length = &s_read_lengths[kind - 1];
            if (ctxt->offset == 0) {
                *length = 0;
                system_state_t state;
                system_state_snapshot(&state);
                if (kind == BLE_BUFFER_RAW) {
                    imu_raw_sample_t sample;
                    if (imu_manager_latest_raw(&sample))
                        *length = ble_batch_encode_raw(packet, BLE_BATCH_MAX_FRAME_SIZE, &sample, 1);
                } else if (kind == BLE_BUFFER_DMP) {
                    imu_dmp_sample_t sample;
                    if (imu_manager_latest_dmp(&sample))
                        *length = ble_batch_encode_dmp(packet, BLE_BATCH_MAX_FRAME_SIZE, &sample, 1,
                                                       state.actual_dmp_sample_rate_hz);
                } else if (kind == BLE_BUFFER_AUX) {
                    adc_sample_t sample;
                    if (adc_manager_latest(&sample))
                        *length = ble_batch_encode_aux(packet, BLE_BATCH_MAX_FRAME_SIZE, &sample, 1,
                                                       state.actual_adc_sample_rate_hz);
                } else {
                    *length = ble_batch_encode_status(packet, BLE_BATCH_MAX_FRAME_SIZE, &state);
                }
            }
            if (!*length) return BLE_ATT_ERR_UNLIKELY;
            return append_read_response(ctxt->om, packet, *length);
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
        case CONTROL_SET_ADC_RATE:
#if CONFIG_AUX_ADC_ENABLED
            if (len != 3 || value < 1 || value > 200)
                return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
            SYSTEM_STATE_SET(requested_adc_rate_hz, value);
            break;
#else
            return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
#endif
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
        ble_sample_buffer_set_enabled(BLE_BUFFER_DMP, enabled);
        atomic_store(&s_dmp_notifications_enabled, enabled);
        SYSTEM_STATE_SET(imu_dmp_subscribed, enabled);
    } else if (attr == gatt_imu_raw_handle) {
        ble_sample_buffer_set_enabled(BLE_BUFFER_RAW, enabled);
        atomic_store(&s_raw_notifications_enabled, enabled);
        SYSTEM_STATE_SET(imu_raw_subscribed, enabled);
    } else if (attr == gatt_aux_handle) {
        ble_sample_buffer_set_enabled(BLE_BUFFER_AUX, enabled);
        atomic_store(&s_aux_notifications_enabled, enabled);
        SYSTEM_STATE_SET(aux_subscribed, enabled);
    } else if (attr == gatt_status_handle) {
        if (atomic_load(&s_status_notifications_enabled) != enabled)
            atomic_fetch_add(&s_status_epoch, 1);
        atomic_store(&s_status_notifications_enabled, enabled);
    } else if (attr == gatt_battery_handle) {
        if (atomic_load(&s_battery_notifications_enabled) != enabled)
            atomic_fetch_add(&s_battery_epoch, 1);
        atomic_store(&s_battery_notifications_enabled, enabled);
    }
}

void gatt_clear_subscriptions(void)
{
    ble_sample_buffer_disable_all();
    memset(s_read_lengths, 0, sizeof(s_read_lengths));
    atomic_fetch_add(&s_status_epoch, 1);
    atomic_fetch_add(&s_battery_epoch, 1);
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

uint32_t gatt_stream_epoch(unsigned kind)
{
    if (kind >= 1 && kind <= 3) return ble_sample_buffer_epoch(kind);
    if (kind == 4) return atomic_load(&s_status_epoch);
    if (kind == 5) return atomic_load(&s_battery_epoch);
    return 0;
}

bool gatt_stream_subscribed(unsigned kind)
{
    switch (kind) {
    case BLE_BUFFER_RAW: return gatt_imu_raw_subscribed();
    case BLE_BUFFER_DMP: return gatt_dmp_subscribed();
    case BLE_BUFFER_AUX: return gatt_aux_subscribed();
    case 4: return gatt_status_subscribed();
    case 5: return gatt_battery_subscribed();
    default: return false;
    }
}
