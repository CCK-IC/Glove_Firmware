/* Exercise the actual ADC manager through SDK seams, without copying its logic.
 * Each scenario gets a fresh process, matching its single-init lifecycle.
 * GPIO mapping below is a deterministic host model, not a wiring claim. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "adc/adc_manager.h"
#include "board/board_config.h"
#include "freertos/task.h"
#include "system_state.h"

_Static_assert(APP_AUX_ADC_CHANNEL_COUNT == TEST_EXPECTED_CHANNEL_COUNT,
               "Active count must follow configured count or disabled=0");

static int64_t s_now_us = 1000000;
static void (*s_task)(void *);
static void *s_task_arg;
static jmp_buf s_iteration_end;
static unsigned s_map_calls, s_unit_calls, s_config_calls, s_read_calls;
static unsigned s_channel_reads[8];
static unsigned s_read_failure, s_generation;
static bool s_map_failure, s_unit_failure, s_config_failure, s_task_failure;
static int s_unit_token;
static int s_mapped_gpios[8];

int64_t esp_timer_get_time(void) { return s_now_us; }
TickType_t xTaskGetTickCount(void) { return (TickType_t)(s_now_us / 1000); }

int xTaskCreate(void (*task)(void *), const char *name, unsigned stack,
                void *arg, unsigned priority, void *handle)
{
    (void)name; (void)stack; (void)priority; (void)handle;
    assert(!s_task);
    if (s_task_failure) return 0;
    s_task = task;
    s_task_arg = arg;
    return pdPASS;
}

void vTaskDelayUntil(TickType_t *last, TickType_t period)
{
    assert(period > 0);
    *last += period;
    s_now_us += (int64_t)period * 1000;
    /* The production task has published/unlocked before reaching this seam. */
    longjmp(s_iteration_end, 1);
}

esp_err_t adc_oneshot_io_to_channel(int gpio, adc_unit_t *unit, adc_channel_t *channel)
{
    assert((int)s_map_calls < TEST_EXPECTED_CHANNEL_COUNT);
    unsigned slot = s_map_calls++;
    if (gpio < 0 || gpio == 48 ||
        (s_map_failure && slot + 1 == TEST_EXPECTED_CHANNEL_COUNT))
        return ESP_ERR_INVALID_ARG;
    s_mapped_gpios[slot] = gpio;
    *unit = 0;
    *channel = (int)slot;
    return ESP_OK;
}

esp_err_t board_adc_get_unit(adc_unit_t unit, adc_oneshot_unit_handle_t *out)
{
    assert(unit == 0);
    assert(s_map_calls == TEST_EXPECTED_CHANNEL_COUNT);
    ++s_unit_calls;
    if (s_unit_failure) return ESP_FAIL;
    *out = &s_unit_token;
    return ESP_OK;
}

esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t unit,
                                   adc_channel_t channel, const adc_oneshot_chan_cfg_t *cfg)
{
    assert(unit == &s_unit_token);
    assert(channel == (int)s_config_calls);
    assert(cfg->bitwidth == 12);
    assert(cfg->atten == ADC_ATTEN_DB_12);
    ++s_config_calls;
    return s_config_failure ? ESP_FAIL : ESP_OK;
}

esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t unit, adc_channel_t channel, int *raw)
{
    assert(unit == &s_unit_token);
    assert(channel >= 0 && channel < TEST_EXPECTED_CHANNEL_COUNT);
    ++s_read_calls;
    if (s_read_calls == s_read_failure) return ESP_FAIL;
    /* Average 100/102 -> 101, 200/202 -> 201, etc. Vary later iterations. */
    *raw = (channel + 1) * 100 + (int)s_generation * 10 +
           (int)(s_channel_reads[channel]++ % 2) * 2;
    return ESP_OK;
}

static void run_iteration(void)
{
    assert(s_task);
    s_read_calls = 0;
    memset(s_channel_reads, 0, sizeof(s_channel_reads));
    if (setjmp(s_iteration_end) == 0) {
        s_task(s_task_arg);
        assert(!"ADC task unexpectedly returned");
    }
}

static void check_sample(const adc_sample_t *sample, unsigned sequence, uint32_t timestamp)
{
    assert(sample->sequence == sequence);
    assert(sample->timestamp_ms == timestamp);
    for (unsigned i = 0; i < 8; ++i) {
        int expected = 0;
#if TEST_EXPECTED_CHANNEL_COUNT > 0
        if (i < TEST_EXPECTED_CHANNEL_COUNT) {
#if CONFIG_ADC_DUMMY
            expected = (int)(i + 1) * 0x100;
#else
            expected = (int)(i + 1) * 100 + (int)s_generation * 10 + 1;
#endif
        }
#endif
        assert(sample->values[i] == expected);
    }
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *scenario = argv[1];
    assert(system_state_init() == ESP_OK);
    adc_sample_t sample;
    assert(!adc_manager_latest(NULL));
    assert(!adc_manager_latest(&sample));
#if CONFIG_AUX_ADC_ENABLED
    assert(adc_manager_start() == ESP_ERR_INVALID_STATE);
#endif
    s_map_failure = strcmp(scenario, "map_fail") == 0;
    s_unit_failure = strcmp(scenario, "unit_fail") == 0;
    s_config_failure = strcmp(scenario, "config_fail") == 0;
    s_task_failure = strcmp(scenario, "task_fail") == 0;
    esp_err_t result = adc_manager_init();
    if (strcmp(scenario, "invalid_gpio") == 0 || s_map_failure) {
        assert(result == ESP_ERR_INVALID_ARG);
        assert(s_unit_calls == 0 && s_config_calls == 0 && !s_task);
        puts("PASS: invalid mapping rejected before hardware setup");
        return 0;
    }
    if (s_unit_failure || s_config_failure) {
        assert(result == ESP_FAIL && !s_task);
        assert(s_unit_calls == 1);
        assert(s_config_calls == (s_config_failure ? 1U : 0U));
        puts("PASS: ADC setup failure propagated");
        return 0;
    }
    assert(result == ESP_OK);
#if CONFIG_AUX_ADC_ENABLED && !CONFIG_ADC_DUMMY
    assert(s_map_calls == TEST_EXPECTED_CHANNEL_COUNT);
    assert(s_unit_calls == TEST_EXPECTED_CHANNEL_COUNT);
    assert(s_config_calls == TEST_EXPECTED_CHANNEL_COUNT);
    for (unsigned i = 0; i < TEST_EXPECTED_CHANNEL_COUNT; ++i)
        for (unsigned j = 0; j < i; ++j) assert(s_mapped_gpios[i] != s_mapped_gpios[j]);
#else
    assert(s_map_calls == 0 && s_unit_calls == 0 && s_config_calls == 0);
#endif
    assert(!adc_manager_latest(&sample));
    if (s_task_failure) {
        assert(adc_manager_start() == ESP_ERR_NO_MEM && !s_task);
        puts("PASS: task creation failure propagated");
        return 0;
    }
    assert(adc_manager_start() == ESP_OK);
#if !CONFIG_AUX_ADC_ENABLED
    assert(!s_task && !adc_manager_latest(&sample) && s_read_calls == 0);
    puts("PASS: disabled ADC allocates/configures/starts/reads nothing");
    return 0;
#else
    uint32_t timestamp = (uint32_t)(s_now_us / 1000);
    run_iteration();
    assert(adc_manager_latest(&sample));
    check_sample(&sample, 1, timestamp);
#if CONFIG_ADC_DUMMY
    assert(s_read_calls == 0);
#else
    assert(s_read_calls == TEST_EXPECTED_CHANNEL_COUNT * 2);
#endif
    system_state_t state;
    system_state_snapshot(&state);
    assert(state.adc_sample_count == 1);
#if !CONFIG_ADC_DUMMY
    if (strncmp(scenario, "read_fail_", 10) == 0) {
        adc_sample_t previous = sample;
        s_read_failure = strcmp(scenario, "read_fail_first") == 0 ? 1 :
                         TEST_EXPECTED_CHANNEL_COUNT * 2;
        ++s_generation;
        run_iteration();
        assert(s_read_calls == s_read_failure);
        assert(adc_manager_latest(&sample));
        assert(memcmp(&sample, &previous, sizeof(sample)) == 0);
        system_state_snapshot(&state);
        assert(state.adc_sample_count == 1);
        assert(state.last_error == ESP_FAIL && strcmp(state.last_error_module, "aux_adc") == 0);
        s_read_failure = 0;
    }
#endif
    ++s_generation;
    timestamp = (uint32_t)(s_now_us / 1000);
    run_iteration();
    assert(adc_manager_latest(&sample));
    check_sample(&sample, 2, timestamp);
    system_state_snapshot(&state);
    assert(state.adc_sample_count == 2);
    puts("PASS: active channels, averaging/dummy values, zero padding, sequence and recovery");
    return 0;
#endif
}
