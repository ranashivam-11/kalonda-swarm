/*
 * hal_store.c - NVS persistence: parameters, yaw look-up table, boot counter.
 *
 * NVS keys are limited to 15 characters, so parameters are stored as one
 * versioned blob rather than key-per-parameter. The LUT is what the array
 * has *learned*; losing it to a brown-out would throw away days of trials,
 * so it is written with a version and sector count and rejected on mismatch.
 */
#include <string.h>

#include "esp_log.h"
#include "hal.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "store";
static const char *NS = "ks";

#define PARAMS_VERSION 1u
#define LUT_VERSION 2u /* v2: KS_BINS = sectors x stability classes */

typedef struct {
    uint32_t version;
    uint32_t size;
    ks_params_t p;
} params_blob_t;

typedef struct {
    float off_deg, step_deg;
    int8_t dir;
    uint8_t pad[3];
    float led_self_w, led_down_w, recv_w;
    uint16_t n_trials, n_accept;
} lut_item_t;

typedef struct {
    uint32_t version;
    uint32_t sectors;
    lut_item_t e[KS_BINS];
} lut_blob_t;

esp_err_t hal_store_init(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS layout changed, erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    return e;
}

uint16_t hal_store_bump_boot(void)
{
    nvs_handle_t h;
    uint16_t boot = 0;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return 0;
    (void)nvs_get_u16(h, "boot", &boot);
    boot = (uint16_t)(boot + 1u);
    nvs_set_u16(h, "boot", boot);
    nvs_commit(h);
    nvs_close(h);
    return boot;
}

bool hal_store_load_params(ks_params_t *p)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return false;
    static params_blob_t b;
    size_t len = sizeof b;
    const esp_err_t e = nvs_get_blob(h, "params", &b, &len);
    nvs_close(h);
    if (e != ESP_OK || len != sizeof b || b.version != PARAMS_VERSION || b.size != sizeof(ks_params_t)) {
        return false;
    }
    *p = b.p;
    return true;
}

esp_err_t hal_store_save_params(const ks_params_t *p)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    static params_blob_t b;
    b.version = PARAMS_VERSION;
    b.size = sizeof(ks_params_t);
    b.p = *p;
    e = nvs_set_blob(h, "params", &b, sizeof b);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

void hal_store_load_lut(ks_node_t *n)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return;
    static lut_blob_t b;
    size_t len = sizeof b;
    const esp_err_t e = nvs_get_blob(h, "lut", &b, &len);
    nvs_close(h);
    if (e != ESP_OK || len != sizeof b || b.version != LUT_VERSION || b.sectors != KS_BINS) return;
    for (int s = 0; s < KS_BINS; ++s) {
        ks_lut_e_t *d = &n->lut[s];
        ks_node_lut_set(n, (uint8_t)s, b.e[s].off_deg);
        d->step_deg = b.e[s].step_deg;
        d->dir = b.e[s].dir ? b.e[s].dir : 1;
        d->led_self_w = b.e[s].led_self_w;
        d->led_down_w = b.e[s].led_down_w;
        d->recv_w = b.e[s].recv_w;
        d->n_trials = b.e[s].n_trials;
        d->n_accept = b.e[s].n_accept;
    }
    ESP_LOGI(TAG, "restored learned yaw table");
}

esp_err_t hal_store_save_lut(const ks_node_t *n)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    static lut_blob_t b;
    memset(&b, 0, sizeof b);
    b.version = LUT_VERSION;
    b.sectors = KS_BINS;
    for (int s = 0; s < KS_BINS; ++s) {
        const ks_lut_e_t *d = &n->lut[s];
        b.e[s].off_deg = d->off_deg;
        b.e[s].step_deg = d->step_deg;
        b.e[s].dir = d->dir;
        b.e[s].led_self_w = d->led_self_w;
        b.e[s].led_down_w = d->led_down_w;
        b.e[s].recv_w = d->recv_w;
        b.e[s].n_trials = d->n_trials;
        b.e[s].n_accept = d->n_accept;
    }
    e = nvs_set_blob(h, "lut", &b, sizeof b);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}
