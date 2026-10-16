#include "led_manager.h"

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
    uint8_t r, g, b;
    uint16_t on_ms, off_ms, rest_ms;
    uint8_t priority;
    uint8_t repeats;
    uint32_t expires_ms;
} pattern_t;

static QueueHandle_t s_queue;
static led_strip_handle_t s_strip;
static led_pattern_id_t s_current;
static pattern_t s_slots[8];

static pattern_t definition(led_pattern_id_t id)
{
    switch (id) {
    case LED_PATTERN_POWER_OFF:
        return (pattern_t){id,255,80,0,150,150,0,100,0,0};
    case LED_PATTERN_PAIRING:
        return (pattern_t){id,0,255,255,200,200,0,90,0,0};
    case LED_PATTERN_LOW_BATTERY:
        return (pattern_t){id,255,0,0,150,850,2000,80,0,0};
    case LED_PATTERN_CONNECTED:
        return (pattern_t){id,0,0,255,100,1900,0,60,0,0};
    case LED_PATTERN_DISCONNECTED:
        return (pattern_t){id,255,160,0,150,850,0,50,0,0};
    case LED_PATTERN_DEBUG_FLASH:
        return (pattern_t){id,0,255,0,150,100,0,40,1,500};
    case LED_PATTERN_TEST:
        return (pattern_t){id,255,255,255,250,250,0,30,0,0};
    default:
        return (pattern_t){id,0,0,0,0,1000,0,0,0,0};
    }
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
        led_strip_set_pixel(s_strip, 0, r, g, b);
        led_strip_refresh(s_strip);
    }
}

static pattern_t select_pattern(void)
{
    pattern_t best = definition(LED_PATTERN_IDLE);
    uint32_t now = esp_timer_get_time() / 1000ULL;

    for (unsigned i = 0; i < sizeof(s_slots) / sizeof(s_slots[0]); ++i) {
        pattern_t *p = &s_slots[i];
        if (p->id == LED_PATTERN_IDLE) continue;
        if (p->expires_ms && now >= p->expires_ms) {
            p->id = LED_PATTERN_IDLE;
            continue;
        }
        if (p->priority > best.priority) best = *p;
    }
    return best;
}

static void task_main(void *arg)
{
    (void)arg;
    bool on = false;
    int64_t next = 0;

    while (true) {
        pattern_t request;
        if (xQueueReceive(s_queue, &request, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_slots[request.id] = request;
            if (request.expires_ms)
                s_slots[request.id].expires_ms +=
                    esp_timer_get_time() / 1000ULL;
        }

        pattern_t p = select_pattern();
        s_current = p.id;
        int64_t now = esp_timer_get_time() / 1000ULL;

        if (p.id == LED_PATTERN_IDLE) {
            set_rgb(0, 0, 0);
            on = false;
            next = now + 100;
        } else if (now >= next) {
            on = !on;
            set_rgb(on ? p.r : 0, on ? p.g : 0, on ? p.b : 0);
            next = now + (on ? p.on_ms : p.off_ms);
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

    s_queue = xQueueCreate(8, sizeof(pattern_t));
    if (!s_queue) return ESP_ERR_NO_MEM;

    return xTaskCreate(task_main, "led", 3072, NULL, 3, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t led_set_pattern(led_pattern_id_t id)
{
    if (!s_queue || id >= 8) return ESP_ERR_INVALID_STATE;
    pattern_t p = definition(id);
    return xQueueSend(s_queue, &p, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t led_clear_pattern(led_pattern_id_t id)
{
    if (!s_queue || id >= 8) return ESP_ERR_INVALID_STATE;
    pattern_t p = {.id = id};
    return xQueueSend(s_queue, &p, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

led_pattern_id_t led_get_current_pattern(void)
{
    return s_current;
}