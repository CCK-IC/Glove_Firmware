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

    struct { uint8_t before[8], packet[AUX_PACKET_V1_SIZE], after[8]; } guarded;
    memset(&guarded, 0xA5, sizeof(guarded));
    adc_sample_t sample = {.sequence = 0x123, .timestamp_ms = 0x12345678,
        .values = {-32768, -1, 0, 1, 255, 256, 32767, 0x1234}};
    /* A default ATT MTU cannot carry V1; refusal must not touch the buffer. */
    assert(ble_encode_aux_for_mtu(guarded.packet, sizeof(guarded.packet), 23, &sample) == 0);
    for (unsigned i = 0; i < sizeof(guarded.packet); ++i) assert(guarded.packet[i] == 0xA5);
    assert(ble_encode_aux_for_mtu(guarded.packet, sizeof(guarded.packet) - 1, 25, &sample) == 0);
    assert(ble_encode_aux_for_mtu(NULL, 22, 25, &sample) == 0);
    assert(ble_encode_aux_for_mtu(guarded.packet, 22, 25, NULL) == 0);
    assert(ble_encode_aux_for_mtu(guarded.packet, 22, 25, &sample) == 22);
    assert(guarded.packet[0] == 1 && guarded.packet[1] == 0x23);
    assert(guarded.packet[2] == 0x78 && guarded.packet[5] == 0x12);
    assert(guarded.packet[6] == 0 && guarded.packet[7] == 0x80);
    assert(guarded.packet[20] == 0x34 && guarded.packet[21] == 0x12);
    for (unsigned i = 0; i < 8; ++i) {
        assert(guarded.before[i] == 0xA5);
        assert(guarded.after[i] == 0xA5);
    }
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
