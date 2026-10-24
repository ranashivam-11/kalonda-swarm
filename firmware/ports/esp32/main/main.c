/*
 * main.c - Kalonda swarm node, ESP32 port.
 *
 * One FreeRTOS task (core 1, 10 Hz) owns the node: drain radio -> read
 * sensors -> ks_node_step() -> transmit, log, drive yaw. Wi-Fi/ESP-NOW runs
 * on core 0. There is no central controller anywhere in the array: every
 * node runs exactly this loop.
 */
#include <string.h>

#include "board.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal.h"

static const char *TAG = "kalonda";

static ks_params_t s_params;     /* edited by the console, applied on reboot */
static ks_node_t s_node;
static ks_outputs_t s_out;
static volatile bool s_reboot;

#define LUT_PERSIST_MIN_MS 600000u /* flash wear: at most every 10 min */

static uint16_t id_from_mac(void)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    return (uint16_t)(((uint16_t)mac[4] << 8 | mac[5]) % 65000u);
}

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void node_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    TickType_t last = xTaskGetTickCount();
    float yaw_meas = 0.0f;
    bool yaw_stall = false;
    bool persist_pending = false;
    uint32_t last_persist = now_ms();

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(KS_STEP_MS));
        const uint32_t t = now_ms();

        ks_frame_t f;
        uint32_t t_rx;
        while (hal_radio_pop(&f, &t_rx)) ks_node_rx(&s_node, f.data, f.len, f.rssi, t_rx);

        ks_sensors_t s;
        hal_sensors_read(&s, t);
        s.yaw_deg = yaw_meas;
        if (yaw_stall) s.driver_faults |= KS_F_YAW_STALL;

        ks_node_step(&s_node, &s, &s_out);

        for (uint8_t i = 0; i < s_out.n_tx; ++i) hal_radio_send(s_out.tx[i].data, s_out.tx[i].len);
        for (uint8_t i = 0; i < s_out.n_log; ++i) hal_log_write(&s_out.log[i]);

        yaw_meas = hal_yaw_update(s_out.yaw_cmd_deg, s_out.yaw_enable, t, &yaw_stall);
        hal_brake(s_out.brake);
        /* slow blink when negotiating normally, fast blink otherwise */
        gpio_set_level(PIN_LED, (int)((t / (s_out.mode == KS_MODE_NORMAL ? 1000u : 150u)) & 1u));

        if (s_out.persist_lut) persist_pending = true;
        if (persist_pending && (uint32_t)(t - last_persist) >= LUT_PERSIST_MIN_MS) {
            hal_store_save_lut(&s_node);
            last_persist = t;
            persist_pending = false;
        }
        if (s_reboot) {
            hal_store_save_lut(&s_node);
            esp_restart();
        }
        esp_task_wdt_reset();
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(hal_store_init());
    const uint16_t boot = hal_store_bump_boot();

    ks_params_default(&s_params);
    if (!hal_store_load_params(&s_params)) {
        /* Unprovisioned: unknown position, default key. Operate greedily and
         * never propose a yaw trial until a technician runs "save". */
        s_params.node_id = id_from_mac();
        s_params.yaw_offset_max_deg = 0.0f;
        s_params.step_max_deg = 0.0f;
        s_params.step_init_deg = 0.0f;
        s_params.step_min_deg = 0.0f;
        ESP_LOGW(TAG, "unprovisioned node %u: greedy, listen-only steering", s_params.node_id);
    } else if (ks_params_validate(&s_params) != 0) {
        ESP_LOGE(TAG, "stored parameters inconsistent (rule %d): factory defaults",
                 ks_params_validate(&s_params));
        const uint16_t id = s_params.node_id;
        ks_params_default(&s_params);
        s_params.node_id = id;
    }

    console_start(&s_params, &s_reboot);
    ESP_ERROR_CHECK(hal_log_init());
    ESP_ERROR_CHECK(hal_sensors_init(s_params.pole_pairs, s_params.vane_north_deg));
    ESP_ERROR_CHECK(hal_yaw_init(s_params.yaw_north_deg));
    gpio_reset_pin(PIN_LED);
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(hal_radio_init());

    ks_node_init(&s_node, &s_params, boot, now_ms());
    hal_store_load_lut(&s_node);

    ks_logrec_t r;
    ks_log_event_t ev;
    memset(&ev, 0, sizeof ev);
    ev.code = KS_EV_BOOT;
    ev.arg = boot;
    ev.u[0] = (uint32_t)esp_reset_reason();
    ks_log_event(&r, s_params.node_id, now_ms(), 0u, &ev);
    hal_log_write(&r);

    ESP_LOGI(TAG, "node %u boot %u at (%.1f, %.1f) m, utility %s beta %.2f", s_params.node_id, boot,
             (double)s_params.x_m, (double)s_params.y_m,
             s_params.utility == KS_UTIL_SELFISH ? "selfish" : "cooperative", (double)s_params.beta);
    xTaskCreatePinnedToCore(node_task, "node", 8192, NULL, 5, NULL, 1);
}
