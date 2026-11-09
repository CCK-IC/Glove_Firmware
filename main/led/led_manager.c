#include "led_manager.h"

#include <stdatomic.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "led_strip.h"
#include "board/board_pins.h"

static const char *TAG = "led";

typedef struct {
    led_pattern_id_t id;
    uint8_t red, green, blue;
    uint16_t on_ms, off_ms;
    uint8_t priority;
    uint32_t duration_ms;
    int64_t expires_at_ms;
} led_pattern_t;

static QueueHandle_t s_queue;
static led_strip_handle_t s_strip;
static atomic_int s_current_pattern = LED_PATTERN_IDLE;
static led_pattern_t s_active_patterns[LED_PATTERN_COUNT];

static led_pattern_t pattern_definition(led_pattern_id_t id)
{
    led_pattern_t pattern = {.id = id};
    switch (id) {
    case LED_PATTERN_POWER_OFF:
        pattern = (led_pattern_t){.id=id, .red=255, .green=80,
            .on_ms=150, .off_ms=150, .priority=100};
        break;
    case LED_PATTERN_PAIRING:
        pattern = (led_pattern_t){.id=id, .green=255, .blue=255,
            .on_ms=200, .off_ms=200, .priority=90};
        break;
    case LED_PATTERN_LOW_BATTERY:
        pattern = (led_pattern_t){.id=id, .red=255,
            .on_ms=150, .off_ms=850, .priority=80};
        break;
    case LED_PATTERN_CONNECTED:
        pattern = (led_pattern_t){.id=id, .blue=255,
            .on_ms=100, .off_ms=1900, .priority=60};
        break;
    case LED_PATTERN_DISCONNECTED:
        pattern = (led_pattern_t){.id=id, .red=255, .green=160,
            .on_ms=150, .off_ms=850, .priority=50};
        break;
    case LED_PATTERN_DEBUG_FLASH:
        pattern = (led_pattern_t){.id=id, .green=255,
            .on_ms=150, .off_ms=100, .priority=70, .duration_ms=500};
        break;
    case LED_PATTERN_TEST:
        pattern = (led_pattern_t){.id=id, .red=255, .green=255, .blue=255,
            .on_ms=250, .off_ms=250, .priority=95};
        break;
    default:
        break;
    }
    return pattern;
}

static void set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    const uint8_t limit = CONFIG_LED_BRIGHTNESS_LIMIT;
    r = (uint16_t)r * limit / 255;
    g = (uint16_t)g * limit / 255;
    b = (uint16_t)b * limit / 255;

#if CONFIG_LED_ORDER_GRB
    uint8_t tmp = r; r = g; g = tmp;
#elif CONFIG_LED_ORDER_BRG
    uint8_t nr = b, ng = r, nb = g; r = nr; g = ng; b = nb;
#endif

    if (s_strip) {
        static bool color_valid;
        static uint8_t previous_red, previous_green, previous_blue;
        if (color_valid && r == previous_red && g == previous_green && b == previous_blue)
            return;
        esp_err_t err = led_strip_set_pixel(s_strip, 0, r, g, b);
        if (err == ESP_OK) err = led_strip_refresh(s_strip);
        if (err == ESP_OK) {
            previous_red = r;
            previous_green = g;
            previous_blue = b;
            color_valid = true;
        }
    }
}

static led_pattern_t select_pattern(void)
{
    led_pattern_t best = pattern_definition(LED_PATTERN_IDLE);
    int64_t now = esp_timer_get_time() / 1000;

    for (unsigned i = 0; i < LED_PATTERN_COUNT; ++i) {
        led_pattern_t *pattern = &s_active_patterns[i];
        if (pattern->id == LED_PATTERN_IDLE) continue;
        if (pattern->expires_at_ms && now >= pattern->expires_at_ms) {
            pattern->id = LED_PATTERN_IDLE;
            continue;
        }
        if (pattern->priority > best.priority) best = *pattern;
    }
    return best;
}

static void led_pattern_task(void *arg)
{
    (void)arg;
    bool on = false;
    int64_t next_transition_ms = 0;
    led_pattern_id_t previous_pattern = LED_PATTERN_IDLE;

    while (true) {
        led_pattern_t request;
        if (xQueueReceive(s_queue, &request, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_active_patterns[request.id] = request;
            if (request.duration_ms)
                s_active_patterns[request.id].expires_at_ms =
                    esp_timer_get_time() / 1000 + request.duration_ms;
        }

        led_pattern_t selected = select_pattern();
        atomic_store(&s_current_pattern, selected.id);
        int64_t now = esp_timer_get_time() / 1000;
        if (selected.id != previous_pattern) {
            /* A higher-priority indication must not wait for an old off phase. */
            on = false;
            next_transition_ms = now;
            previous_pattern = selected.id;
        }

        if (selected.id == LED_PATTERN_IDLE) {
            set_rgb(0, 0, 0);
            on = false;
            next_transition_ms = now + 100;
        } else if (now >= next_transition_ms) {
            on = !on;
            set_rgb(on ? selected.red : 0, on ? selected.green : 0, on ? selected.blue : 0);
            next_transition_ms = now + (on ? selected.on_ms : selected.off_ms);
        }
    }
}

esp_err_t led_manager_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = BOARD_LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LED strip init failed: %s", esp_err_to_name(err));
        return err;
    }

    s_queue = xQueueCreate(LED_PATTERN_COUNT, sizeof(led_pattern_t));
    if (!s_queue) {
        led_strip_del(s_strip);
        s_strip = NULL;
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(led_pattern_task, "led", 3072, NULL, 3, NULL) != pdPASS) {
        vQueueDelete(s_queue);
        s_queue = NULL;
        led_strip_del(s_strip);
        s_strip = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t led_set_pattern(led_pattern_id_t id)
{
    if ((unsigned)id >= LED_PATTERN_COUNT) return ESP_ERR_INVALID_ARG;
    if (!s_queue) return ESP_ERR_INVALID_STATE;
    led_pattern_t pattern = pattern_definition(id);
    return xQueueSend(s_queue, &pattern, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t led_clear_pattern(led_pattern_id_t id)
{
    if ((unsigned)id >= LED_PATTERN_COUNT) return ESP_ERR_INVALID_ARG;
    if (!s_queue) return ESP_ERR_INVALID_STATE;
    led_pattern_t pattern = {.id = id};
    return xQueueSend(s_queue, &pattern, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

led_pattern_id_t led_get_current_pattern(void)
{
    return atomic_load(&s_current_pattern);
}
