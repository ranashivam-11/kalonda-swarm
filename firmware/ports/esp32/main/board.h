/*
 * board.h - reference node wiring (ESP32-WROOM-32E) and calibration defaults.
 *
 * All analogue inputs are on ADC1: ADC2 is unusable while Wi-Fi (and hence
 * ESP-NOW) is running. GPIO 34-39 are input-only and have no pull-ups, so
 * open-collector Hall switches on them need external 10 k pull-ups.
 * See docs/hardware.md for the schematic-level description and BOM.
 */
#ifndef BOARD_H
#define BOARD_H

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

/* --- digital inputs (edge-timed in an ISR) ------------------------------ */
#define PIN_HALL_A GPIO_NUM_26      /* rotor Hall switch, sees rotor magnets   */
#define PIN_HALL_B GPIO_NUM_27      /* second Hall, 90 el. deg: plausibility   */
#define PIN_ANEMO GPIO_NUM_14       /* cup anemometer reed/Hall pulse          */

/* --- ADC1 channels ------------------------------------------------------- */
#define ADC_CH_VBUS ADC_CHANNEL_0   /* GPIO36: bus divider 100k/4k7            */
#define ADC_CH_IBUS ADC_CHANNEL_3   /* GPIO39: INA181A2 (G=50) on 10 mOhm shunt */
#define ADC_CH_VANE ADC_CHANNEL_6   /* GPIO34: vane potentiometer wiper        */
#define ADC_CH_NTC_STATOR ADC_CHANNEL_7 /* GPIO35: 10k NTC epoxied in a slot   */
#define ADC_CH_NTC_BOARD ADC_CHANNEL_4  /* GPIO32: 10k NTC beside the module    */
#define ADC_CH_VBATT ADC_CHANNEL_5  /* GPIO33: battery divider 100k/10k        */

/* --- outputs -------------------------------------------------------------- */
#define PIN_YAW_IN1 GPIO_NUM_18     /* DRV8871 IN1 (PWM)                       */
#define PIN_YAW_IN2 GPIO_NUM_19     /* DRV8871 IN2 (PWM)                       */
#define PIN_BRAKE GPIO_NUM_23       /* relay: shorts the rectified bus via dump load */
#define PIN_LED GPIO_NUM_2

/* --- I2C: AS5600 magnetic angle sensor on the yaw shaft ----------------- */
#define PIN_I2C_SDA GPIO_NUM_21
#define PIN_I2C_SCL GPIO_NUM_22
#define AS5600_ADDR 0x36

/* --- UART2: binary log stream to a serial SD logger ---------------------- */
#define LOG_UART_TX GPIO_NUM_17
#define LOG_UART_BAUD 460800

/* --- radio ---------------------------------------------------------------- */
#define ESPNOW_CHANNEL 6

/* --- analogue front-end scaling ------------------------------------------ */
#define VBUS_DIV ((100.0f + 4.7f) / 4.7f)   /* V per V at the pin            */
#define IBUS_V_PER_A (50.0f * 0.010f)       /* INA181A2 * shunt               */
#define IBUS_OFFSET_MV 0.0f
#define VBATT_DIV ((100.0f + 10.0f) / 10.0f)
#define VREF_RAIL_MV 3300.0f                /* NTC and vane dividers' supply  */

/* NTC: 10 k at 25 C, beta 3950, 10 k series resistor to 3V3. */
#define NTC_R0 10000.0f
#define NTC_T0 25.0f
#define NTC_BETA 3950.0f
#define NTC_RSERIES 10000.0f

/* Vane potentiometer: 10-degree dead band, codes in millivolts. */
#define VANE_MV_MIN 60u
#define VANE_MV_MAX 3240u
#define VANE_DEADBAND_DEG 10.0f

/* Anemometer: two pulses per revolution; IEC 61400-12-1 style transfer
 * function from the wind-tunnel calibration certificate of each unit. */
#define ANEMO_SLOPE 0.765f
#define ANEMO_OFFSET 0.35f
#define ANEMO_THRESHOLD_HZ 0.2f

/* Yaw drive: worm gear (self-locking), so the motor is unpowered at rest. */
#define YAW_DEADBAND_DEG 1.5f
#define YAW_KP 0.08f              /* duty per degree                         */
#define YAW_DUTY_MIN 0.35f        /* breakaway                               */
#define YAW_DUTY_MAX 0.85f
#define YAW_STALL_MS 3000u
#define YAW_STALL_MIN_DEG 0.5f
#define YAW_STALL_PAUSE_MS 30000u

#endif /* BOARD_H */
