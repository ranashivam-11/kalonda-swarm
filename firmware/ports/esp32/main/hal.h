/*
 * hal.h - ESP32 port of the Kalonda swarm node. Everything hardware-specific
 * lives behind these functions; ks_node_step() never sees a register.
 */
#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "ks/ks_node.h"

/* radio (hal_radio.c) */
esp_err_t hal_radio_init(void);
/* Pop one received frame; returns false when the queue is empty. */
bool hal_radio_pop(ks_frame_t *f, uint32_t *t_ms);
esp_err_t hal_radio_send(const uint8_t *data, uint8_t len);
uint32_t hal_radio_tx_failures(void);

/* sensors (hal_sensors.c) */
esp_err_t hal_sensors_init(float pole_pairs, float vane_north_deg);
void hal_sensors_read(ks_sensors_t *s, uint32_t now_ms);

/* yaw drive (hal_yaw.c) */
esp_err_t hal_yaw_init(float north_offset_deg);
/* Run one control period; returns measured heading, sets *stall. */
float hal_yaw_update(float cmd_deg, bool enable, uint32_t now_ms, bool *stall);
void hal_brake(bool on);

/* persistence (hal_store.c) */
esp_err_t hal_store_init(void);
uint16_t hal_store_bump_boot(void);
bool hal_store_load_params(ks_params_t *p);
esp_err_t hal_store_save_params(const ks_params_t *p);
void hal_store_load_lut(ks_node_t *n);
esp_err_t hal_store_save_lut(const ks_node_t *n);

/* logging (hal_log.c) */
esp_err_t hal_log_init(void);
void hal_log_write(const ks_logrec_t *r);

/* provisioning console on UART0 (console.c) */
void console_start(ks_params_t *p, volatile bool *reboot_requested);

#endif /* HAL_H */
