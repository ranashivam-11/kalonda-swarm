/*
 * hal_log.c - binary log stream on UART2 to a serial SD-card logger.
 *
 * Stream framing: 0xA5 0x5A followed by one 64-byte record (which carries
 * its own CRC-16). The decoder (kalonda.telemetry.binlog) resynchronises on
 * the sync word after any corruption.
 */
#include "board.h"
#include "driver/uart.h"
#include "hal.h"

#define LOG_UART UART_NUM_2

esp_err_t hal_log_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = LOG_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t e = uart_driver_install(LOG_UART, 256, 8192, 0, NULL, 0);
    if (e != ESP_OK) return e;
    e = uart_param_config(LOG_UART, &cfg);
    if (e != ESP_OK) return e;
    return uart_set_pin(LOG_UART, LOG_UART_TX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                        UART_PIN_NO_CHANGE);
}

void hal_log_write(const ks_logrec_t *r)
{
    static const uint8_t sync[2] = {0xA5, 0x5A};
    uart_write_bytes(LOG_UART, sync, sizeof sync);
    uart_write_bytes(LOG_UART, r->b, KS_LOG_REC_BYTES);
}
