/*
 * hal_yaw.c - yaw drive: AS5600 Hall-array angle sensor on the yaw shaft and
 * a worm-geared DC motor on a DRV8871 H-bridge. The worm gear is
 * self-locking, so the bridge is released (coast) whenever the heading is
 * inside the dead band: no holding current, no heat in the enclosure.
 */
#include <math.h>

#include "board.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "hal.h"
#include "ks/ks_stats.h"

static const char *TAG = "yaw";

static i2c_master_dev_handle_t s_as5600;
static float s_north_offset;
static float s_last_deg;
static bool s_have_angle;

/* stall detection */
static uint32_t s_drive_since_ms;
static float s_drive_start_deg;
static bool s_driving;
static uint32_t s_pause_until_ms;

#define PWM_RES LEDC_TIMER_10_BIT
#define PWM_MAX 1023u

esp_err_t hal_yaw_init(float north_offset_deg)
{
    s_north_offset = north_offset_deg;

    i2c_master_bus_config_t bcfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = PIN_I2C_SCL,
        .sda_io_num = PIN_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bcfg, &bus));
    i2c_device_config_t dcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dcfg, &s_as5600));

    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    const int pins[2] = {PIN_YAW_IN1, PIN_YAW_IN2};
    for (int i = 0; i < 2; ++i) {
        ledc_channel_config_t c = {
            .gpio_num = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_BRAKE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    hal_brake(false);
    ESP_LOGI(TAG, "yaw drive ready (north offset %.1f deg)", (double)north_offset_deg);
    return ESP_OK;
}

static bool read_angle(float *deg)
{
    uint8_t reg = 0x0B, st = 0;
    if (i2c_master_transmit_receive(s_as5600, &reg, 1, &st, 1, 10) != ESP_OK) return false;
    if (!(st & 0x20u)) return false; /* MD: magnet not detected */
    uint8_t areg = 0x0E, b[2];
    if (i2c_master_transmit_receive(s_as5600, &areg, 1, b, 2, 10) != ESP_OK) return false;
    const uint16_t raw = (uint16_t)(((b[0] & 0x0Fu) << 8) | b[1]);
    *deg = ks_wrap360((float)raw * (360.0f / 4096.0f) + s_north_offset);
    return true;
}

static void drive(float duty_signed)
{
    const float d = ks_clampf(fabsf(duty_signed), 0.0f, 1.0f);
    const uint32_t v = (uint32_t)(d * (float)PWM_MAX);
    /* DRV8871: IN1 PWM / IN2 low = forward, IN1 low / IN2 PWM = reverse,
     * both low = coast. */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_signed > 0.0f ? v : 0u);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_signed < 0.0f ? v : 0u);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

float hal_yaw_update(float cmd_deg, bool enable, uint32_t now_ms, bool *stall)
{
    *stall = false;
    float meas;
    if (read_angle(&meas)) {
        s_last_deg = meas;
        s_have_angle = true;
    } else if (!s_have_angle) {
        drive(0.0f);
        *stall = true; /* no encoder: report as a yaw fault */
        return NAN;
    }

    if (!enable || (int32_t)(now_ms - s_pause_until_ms) < 0) {
        drive(0.0f);
        s_driving = false;
        return s_last_deg;
    }

    const float err = ks_angdiff(cmd_deg, s_last_deg);
    if (fabsf(err) <= YAW_DEADBAND_DEG) {
        drive(0.0f);
        s_driving = false;
        return s_last_deg;
    }

    float duty = YAW_KP * err;
    const float mag = ks_clampf(fabsf(duty), YAW_DUTY_MIN, YAW_DUTY_MAX);
    duty = duty > 0.0f ? mag : -mag;
    if (!s_driving) {
        s_driving = true;
        s_drive_since_ms = now_ms;
        s_drive_start_deg = s_last_deg;
    } else if ((uint32_t)(now_ms - s_drive_since_ms) > YAW_STALL_MS) {
        if (fabsf(ks_angdiff(s_last_deg, s_drive_start_deg)) < YAW_STALL_MIN_DEG) {
            /* Sand in the yaw bearing or a jammed worm: stop and retry later. */
            drive(0.0f);
            s_driving = false;
            s_pause_until_ms = now_ms + YAW_STALL_PAUSE_MS;
            *stall = true;
            return s_last_deg;
        }
        s_drive_since_ms = now_ms;
        s_drive_start_deg = s_last_deg;
    }
    drive(duty);
    return s_last_deg;
}

void hal_brake(bool on) { gpio_set_level(PIN_BRAKE, on ? 1 : 0); }
