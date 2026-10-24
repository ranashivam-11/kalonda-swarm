/*
 * hal_radio.c - ESP-NOW broadcast transport.
 *
 * ESP-NOW cannot encrypt broadcast frames, so confidentiality is not
 * attempted; integrity comes from the SipHash tag inside every frame.
 * Callbacks run in the Wi-Fi task: they only copy into a queue.
 */
#include <string.h>

#include "board.h"
#include "esp_event.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "hal.h"

static const char *TAG = "radio";
static const uint8_t BCAST[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

typedef struct {
    ks_frame_t f;
    uint32_t t_ms;
} rx_item_t;

static QueueHandle_t s_rxq;
static SemaphoreHandle_t s_tx_done;
static volatile uint32_t s_tx_fail;

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len <= 0 || len > KS_FRAME_MAX) return;
    rx_item_t it;
    it.t_ms = (uint32_t)(esp_timer_get_time() / 1000);
    it.f.len = (uint8_t)len;
    it.f.rssi = (info && info->rx_ctrl) ? (int8_t)info->rx_ctrl->rssi : 0;
    memcpy(it.f.data, data, (size_t)len);
    (void)xQueueSend(s_rxq, &it, 0); /* drop when full: never block Wi-Fi */
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void on_sent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
#else
static void on_sent(const uint8_t *mac, esp_now_send_status_t status)
{
    (void)mac;
#endif
    if (status != ESP_NOW_SEND_SUCCESS) s_tx_fail++;
    xSemaphoreGive(s_tx_done);
}

esp_err_t hal_radio_init(void)
{
    s_rxq = xQueueCreate(KS_RX_QUEUE, sizeof(rx_item_t));
    s_tx_done = xSemaphoreCreateBinary();
    if (!s_rxq || !s_tx_done) return ESP_ERR_NO_MEM;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    /* Long-range PHY trades bit rate for ~+5 dB link budget across rooftops. */
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
        WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_sent));
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof peer);
    memcpy(peer.peer_addr, BCAST, ESP_NOW_ETH_ALEN);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    xSemaphoreGive(s_tx_done);
    ESP_LOGI(TAG, "ESP-NOW up on channel %d", ESPNOW_CHANNEL);
    return ESP_OK;
}

bool hal_radio_pop(ks_frame_t *f, uint32_t *t_ms)
{
    rx_item_t it;
    if (xQueueReceive(s_rxq, &it, 0) != pdTRUE) return false;
    *f = it.f;
    *t_ms = it.t_ms;
    return true;
}

esp_err_t hal_radio_send(const uint8_t *data, uint8_t len)
{
    /* Espressif recommends waiting for the previous send callback before
     * queueing the next frame; 20 ms is ample for a 70-byte broadcast. */
    if (xSemaphoreTake(s_tx_done, pdMS_TO_TICKS(20)) != pdTRUE) {
        s_tx_fail++;
    }
    const esp_err_t e = esp_now_send(BCAST, data, len);
    if (e != ESP_OK) {
        s_tx_fail++;
        xSemaphoreGive(s_tx_done);
    }
    return e;
}

uint32_t hal_radio_tx_failures(void) { return s_tx_fail; }
