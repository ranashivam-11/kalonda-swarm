/*
 * console.c - field provisioning over USB serial (UART0, 115200 8N1).
 *
 *   show                     list every parameter
 *   set <name> <value>       e.g. "set node_id 14", "set x_m 21.5"
 *   key <32 hex digits>      array authentication key
 *   save                     validate and write to NVS
 *   reboot
 *
 * A technician with a phone and a USB-OTG cable can commission a node in
 * the field; no laptop and no Wi-Fi access point are needed.
 */
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal.h"

#define CON_UART UART_NUM_0

static ks_params_t *s_p;
static volatile bool *s_reboot;

static void reply(const char *s) { uart_write_bytes(CON_UART, s, strlen(s)); }

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)tolower((unsigned char)c);
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static void handle(char *line)
{
    char buf[96];
    char *cmd = strtok(line, " \t\r\n");
    if (!cmd) return;
    if (strcmp(cmd, "show") == 0) {
        for (size_t i = 0; i < ks_params_count(); ++i) {
            double v = 0.0;
            ks_params_get(s_p, ks_params_name(i), &v);
            snprintf(buf, sizeof buf, "%s = %.6g\r\n", ks_params_name(i), v);
            reply(buf);
        }
    } else if (strcmp(cmd, "set") == 0) {
        const char *name = strtok(NULL, " \t\r\n");
        const char *val = strtok(NULL, " \t\r\n");
        if (!name || !val) { reply("usage: set <name> <value>\r\n"); return; }
        const int rc = ks_params_set(s_p, name, strtod(val, NULL));
        reply(rc == 0 ? "ok\r\n" : (rc == -1 ? "unknown parameter\r\n" : "out of range\r\n"));
    } else if (strcmp(cmd, "key") == 0) {
        const char *hex = strtok(NULL, " \t\r\n");
        if (!hex || strlen(hex) != 32) { reply("need 32 hex digits\r\n"); return; }
        uint8_t k[16];
        for (int i = 0; i < 16; ++i) {
            const int hi = hexval(hex[2 * i]), lo = hexval(hex[2 * i + 1]);
            if (hi < 0 || lo < 0) { reply("bad hex\r\n"); return; }
            k[i] = (uint8_t)(hi << 4 | lo);
        }
        memcpy(s_p->key, k, 16);
        reply("key set\r\n");
    } else if (strcmp(cmd, "save") == 0) {
        const int v = ks_params_validate(s_p);
        if (v != 0) {
            snprintf(buf, sizeof buf, "rejected: rule %d (see ks_params.c)\r\n", v);
            reply(buf);
            return;
        }
        reply(hal_store_save_params(s_p) == ESP_OK ? "saved\r\n" : "NVS error\r\n");
    } else if (strcmp(cmd, "reboot") == 0) {
        *s_reboot = true;
    } else {
        reply("commands: show | set <name> <value> | key <hex32> | save | reboot\r\n");
    }
}

static void console_task(void *arg)
{
    (void)arg;
    char line[128];
    size_t n = 0;
    for (;;) {
        uint8_t c;
        if (uart_read_bytes(CON_UART, &c, 1, pdMS_TO_TICKS(1000)) != 1) continue;
        if (c == '\r' || c == '\n') {
            line[n] = '\0';
            if (n > 0) handle(line);
            n = 0;
        } else if (n + 1 < sizeof line) {
            line[n++] = (char)c;
        }
    }
}

void console_start(ks_params_t *p, volatile bool *reboot_requested)
{
    s_p = p;
    s_reboot = reboot_requested;
    if (uart_driver_install(CON_UART, 512, 0, 0, NULL, 0) != ESP_OK) return;
    xTaskCreate(console_task, "console", 4096, NULL, 2, NULL);
}
