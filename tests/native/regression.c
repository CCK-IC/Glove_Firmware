#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include "ble/ble_packets.h"
#include "system_state.h"

static int64_t s_now_us = 1000000;
int64_t esp_timer_get_time(void) { return s_now_us; }
uint32_t esp_get_free_heap_size(void) { return 1024; }
uint32_t esp_get_minimum_free_heap_size(void) { return 512; }

static void *samples(void *unused)
{
    (void)unused;
    for (unsigned i = 0; i < 100000; ++i) {
        SYSTEM_STATE_INCREMENT(raw_sample_count);
        SYSTEM_STATE_INCREMENT(adc_sample_count);
    }
    return NULL;
}

static void *claim_disconnected_shutdown(void *unused)
{
    (void)unused;
    return (void *)(uintptr_t)system_state_try_begin_disconnected_shutdown(3, 30000000);
}

int main(void)
{
    assert(system_state_init() == ESP_OK);
    /* Competing increments must survive even when a writer has an old snapshot. */
    system_state_t stale, state;
    system_state_snapshot(&stale);
    pthread_t a, b;
    assert(pthread_create(&a, NULL, samples, NULL) == 0);
    assert(pthread_create(&b, NULL, samples, NULL) == 0);
    system_state_set_ble_connected(true);
    assert(pthread_join(a, NULL) == 0);
    assert(pthread_join(b, NULL) == 0);
    system_state_snapshot(&state);
    assert(state.raw_sample_count == 200000);
    assert(state.adc_sample_count == 200000);
    assert(state.ble_connected);
    system_state_set_error("test_module_long", -7);
    system_state_snapshot(&state);
    assert(state.last_error == -7);
    assert(state.last_error_module[11] == '\0');

    _Static_assert(AUX_PACKET_CHANNEL_COUNT == 8, "AUX V1 keeps eight wire slots");
    _Static_assert(AUX_PACKET_V1_SIZE == 22, "AUX V1 wire size must not shrink");
    _Static_assert(APP_AUX_ADC_CHANNEL_COUNT == TEST_EXPECTED_CHANNEL_COUNT,
                   "Active count must follow config, including disabled=0");
    struct { uint8_t before[8], packet[22], after[8]; } guarded;
    adc_sample_t sample = {.sequence = 0x123, .timestamp_ms = 0x12345678,
        .values = {-32768, -1, 0x1234, 1, 255, 256, 32767, 0x5678}};
    adc_sample_t original = sample;
    uint8_t expected[22] = {1, 0x23, 0x78, 0x56, 0x34, 0x12};
    for (unsigned i = 0; i < 8; ++i) {
        uint16_t value = 0;
#if APP_AUX_ADC_CHANNEL_COUNT > 0
        if (i < APP_AUX_ADC_CHANNEL_COUNT) value = (uint16_t)sample.values[i];
#endif
        expected[6 + i * 2] = (uint8_t)value;
        expected[7 + i * 2] = (uint8_t)(value >> 8);
    }
    /* Every rejected boundary must leave the entire destination untouched. */
    const uint16_t rejected_mtus[] = {0, 22, 23, 24};
    for (unsigned n = 0; n < sizeof(rejected_mtus) / sizeof(rejected_mtus[0]); ++n) {
        memset(&guarded, 0xA5, sizeof(guarded));
        assert(ble_encode_aux_for_mtu(guarded.packet, 22, rejected_mtus[n], &sample) == 0);
        for (unsigned i = 0; i < sizeof(guarded); ++i)
            assert(((uint8_t *)&guarded)[i] == 0xA5);
    }
    const size_t rejected_capacities[] = {0, 21};
    for (unsigned n = 0; n < sizeof(rejected_capacities) / sizeof(rejected_capacities[0]); ++n) {
        assert(ble_encode_aux_for_mtu(guarded.packet, rejected_capacities[n], 25, &sample) == 0);
        for (unsigned i = 0; i < sizeof(guarded); ++i)
            assert(((uint8_t *)&guarded)[i] == 0xA5);
    }
    assert(ble_encode_aux_for_mtu(NULL, 22, 25, &sample) == 0);
    assert(ble_encode_aux_for_mtu(guarded.packet, 22, 25, NULL) == 0);
    for (unsigned i = 0; i < sizeof(guarded); ++i)
        assert(((uint8_t *)&guarded)[i] == 0xA5);
    assert(ble_encode_aux_for_mtu(guarded.packet, 22, 25, &sample) == 22);
    assert(memcmp(guarded.packet, expected, sizeof(expected)) == 0);
    for (unsigned i = 0; i < 8; ++i) {
        assert(guarded.before[i] == 0xA5);
        assert(guarded.after[i] == 0xA5);
    }
    /* GATT reads use the direct encoder: it must mask the same dirty slots. */
    memset(guarded.packet, 0xA5, sizeof(guarded.packet));
    ble_encode_aux(guarded.packet, sample.timestamp_ms, sample.sequence, sample.values);
    assert(memcmp(guarded.packet, expected, sizeof(expected)) == 0);
    assert(memcmp(&sample, &original, sizeof(sample)) == 0);
    for (unsigned i = 0; i < 8; ++i) {
        assert(guarded.before[i] == 0xA5);
        assert(guarded.after[i] == 0xA5);
    }
    uint8_t oversized[24];
    memset(oversized, 0xA5, sizeof(oversized));
    assert(ble_encode_aux_for_mtu(oversized, sizeof(oversized), UINT16_MAX, &sample) == 22);
    assert(memcmp(oversized, expected, sizeof(expected)) == 0);
    assert(oversized[22] == 0xA5 && oversized[23] == 0xA5);
    uint8_t quaternion[IMU_PACKET_COMPACT_SIZE];
    imu_dmp_sample_t dmp = {.q0_q30 = -1073741824};
    ble_encode_imu_compact(quaternion, &dmp, 100, 56);
    assert(quaternion[8] == 0x00 && quaternion[9] == 0xC0);
    /* A connection blocks shutdown even after the original boot deadline. */
    s_now_us = 40000000;
    assert(!system_state_try_begin_disconnected_shutdown(3, 30000000));
    system_state_set_ble_connected(false);
    assert(!system_state_try_begin_disconnected_shutdown(3, 0));
    s_now_us += 29999999;
    assert(!system_state_try_begin_disconnected_shutdown(3, 30000000));
    /* Even a connection entirely between monitor polls resets the interval. */
    system_state_set_ble_connected(true);
    ++s_now_us;
    system_state_set_ble_connected(false);
    s_now_us += 29999999;
    assert(!system_state_try_begin_disconnected_shutdown(3, 30000000));
    /* Repeated failed connections must not extend the disconnected interval. */
    system_state_set_ble_connected(false);
    ++s_now_us;
    void *claimed_a, *claimed_b;
    assert(pthread_create(&a, NULL, claim_disconnected_shutdown, NULL) == 0);
    assert(pthread_create(&b, NULL, claim_disconnected_shutdown, NULL) == 0);
    assert(pthread_join(a, &claimed_a) == 0);
    assert(pthread_join(b, &claimed_b) == 0);
    assert((uintptr_t)claimed_a + (uintptr_t)claimed_b == 1);
    assert(!system_state_try_begin_shutdown(1));
    system_state_snapshot(&state);
    assert(state.power_off_pending && state.power_off_reason == 3);
    puts("PASS: AUX bounds/MTU/fields, concurrent state updates and shutdown timing");
    return 0;
}
