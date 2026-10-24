/*
 * hal_sensors.c - Hall edge timing, ADC1 front end, NTCs, vane.
 */
#include <math.h>
#include <string.h>

#include "board.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hal.h"
#include "ks/ks_sense.h"

static const char *TAG = "sensors";

/* ISRs only timestamp edges into rings; the task feeds ks_rotor_t, so no
 * flash-resident code runs in interrupt context. */
#define EDGE_RING 32u
typedef struct {
    volatile uint32_t t[EDGE_RING];
    volatile uint32_t head;
} edge_ring_t;

static edge_ring_t s_ring_rotor, s_ring_anemo;
static uint32_t s_tail_rotor, s_tail_anemo;
static ks_rotor_t s_rotor;     /* Hall A edges                     */
static ks_rotor_t s_anemo;     /* anemometer edges                 */
static volatile uint32_t s_hall_b_edges;
static float s_edges_per_rev = 8.0f;

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static bool s_cali_ok;

static const ks_anemo_cal_t k_anemo = {ANEMO_SLOPE, ANEMO_OFFSET, ANEMO_THRESHOLD_HZ};
static ks_vane_cal_t k_vane = {VANE_MV_MIN, VANE_MV_MAX, VANE_DEADBAND_DEG, 0.0f};
static const ks_ntc_t k_ntc = {NTC_R0, NTC_T0, NTC_BETA, NTC_RSERIES};

static void IRAM_ATTR ring_push(edge_ring_t *r)
{
    const uint32_t h = r->head;
    r->t[h % EDGE_RING] = (uint32_t)esp_timer_get_time();
    r->head = h + 1u;
}

static void IRAM_ATTR isr_hall_a(void *arg)
{
    (void)arg;
    ring_push(&s_ring_rotor);
}

static void IRAM_ATTR isr_hall_b(void *arg)
{
    (void)arg;
    s_hall_b_edges++;
}

static void IRAM_ATTR isr_anemo(void *arg)
{
    (void)arg;
    ring_push(&s_ring_anemo);
}

static void drain(edge_ring_t *r, uint32_t *tail, ks_rotor_t *dst)
{
    const uint32_t head = r->head;
    if (head - *tail > EDGE_RING) *tail = head - EDGE_RING; /* overrun: keep newest */
    while (*tail != head) {
        ks_rotor_edge(dst, r->t[*tail % EDGE_RING]);
        *tail += 1u;
    }
}

esp_err_t hal_sensors_init(float pole_pairs, float vane_north_deg)
{
    s_edges_per_rev = pole_pairs;
    k_vane.north_offset_deg = vane_north_deg;
    ks_rotor_init(&s_rotor);
    ks_rotor_init(&s_anemo);

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_HALL_A) | (1ULL << PIN_HALL_B) | (1ULL << PIN_ANEMO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, /* plus 10 k external for long cables */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_HALL_A, isr_hall_a, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_HALL_B, isr_hall_b, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ANEMO, isr_anemo, NULL));

    adc_oneshot_unit_init_cfg_t ucfg = {.unit_id = ADC_UNIT_1, .ulp_mode = ADC_ULP_MODE_DISABLE};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &s_adc));
    adc_oneshot_chan_cfg_t ccfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    const adc_channel_t chans[] = {ADC_CH_VBUS, ADC_CH_IBUS, ADC_CH_VANE,
                                   ADC_CH_NTC_STATOR, ADC_CH_NTC_BOARD, ADC_CH_VBATT};
    for (size_t i = 0; i < sizeof chans / sizeof chans[0]; ++i) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, chans[i], &ccfg));
    }
    adc_cali_line_fitting_config_t lcfg = {
        .unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    s_cali_ok = (adc_cali_create_scheme_line_fitting(&lcfg, &s_cali) == ESP_OK);
    if (!s_cali_ok) ESP_LOGW(TAG, "no eFuse ADC calibration: using nominal 12-bit scale");
    return ESP_OK;
}

/* Oversampled read in millivolts; NAN on driver error. */
static float read_mv(adc_channel_t ch)
{
    int acc = 0, n = 0;
    for (int i = 0; i < 8; ++i) {
        int raw;
        if (adc_oneshot_read(s_adc, ch, &raw) == ESP_OK) {
            acc += raw;
            ++n;
        }
    }
    if (n == 0) return NAN;
    const int raw = acc / n;
    if (s_cali_ok) {
        int mv;
        if (adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) return (float)mv;
    }
    return (float)raw * 3100.0f / 4095.0f;
}

void hal_sensors_read(ks_sensors_t *s, uint32_t now_ms)
{
    memset(s, 0, sizeof *s);
    s->t_ms = now_ms;
    const uint32_t now_us = (uint32_t)esp_timer_get_time();

    drain(&s_ring_rotor, &s_tail_rotor, &s_rotor);
    drain(&s_ring_anemo, &s_tail_anemo, &s_anemo);
    s->rpm = ks_rotor_rpm(&s_rotor, now_us, s_edges_per_rev, 2000000u);
    const float f_anemo = ks_rotor_rpm(&s_anemo, now_us, 1.0f, 5000000u) / 60.0f;
    s->u_ms = ks_anemo_speed(&k_anemo, f_anemo);
    s->anemo_valid = true; /* stuck/drift detection lives in the supervisor */

    const float v_bus_mv = read_mv(ADC_CH_VBUS);
    const float i_mv = read_mv(ADC_CH_IBUS);
    s->v_bus = v_bus_mv * 1e-3f * VBUS_DIV;
    s->i_bus = (i_mv - IBUS_OFFSET_MV) * 1e-3f / IBUS_V_PER_A;
    if (s->i_bus < 0.0f) s->i_bus = 0.0f;
    if (v_bus_mv > 3050.0f || i_mv > 3050.0f) s->driver_faults |= KS_F_ADC_RANGE;

    const float vane_mv = read_mv(ADC_CH_VANE);
    s->vane_valid = (vane_mv == vane_mv) &&
                    ks_vane_dir(&k_vane, (uint16_t)vane_mv, &s->dir_deg);

    s->t_stator_c = ks_ntc_temp_c(&k_ntc, read_mv(ADC_CH_NTC_STATOR) / VREF_RAIL_MV);
    s->t_mcu_c = ks_ntc_temp_c(&k_ntc, read_mv(ADC_CH_NTC_BOARD) / VREF_RAIL_MV);
    s->batt_v = read_mv(ADC_CH_VBATT) * 1e-3f * VBATT_DIV;

    /* Hall B must see edges whenever Hall A does: a silent B with a
     * turning rotor means a failed sensor or a broken harness. */
    static uint32_t last_b, last_check_ms;
    if ((uint32_t)(now_ms - last_check_ms) >= 5000u) {
        const uint32_t b = s_hall_b_edges;
        if (s->rpm > 60.0f && b == last_b) s->driver_faults |= KS_F_HALL_TIMEOUT;
        last_b = b;
        last_check_ms = now_ms;
    }
}
