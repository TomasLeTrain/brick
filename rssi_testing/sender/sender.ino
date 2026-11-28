#include <esp_wifi.h>
#include <espnow.h>

uint8_t peer_addr[4];
bool peer_connected = false;

void receive_callback(const esp_now_recv_info_t *rx_info, const uint8_t *data,
                      int data_len) {
  // rx_info->rx_ctrl->rssi, rx_info->rx_ctrl->timestamp
}
void send_callback(const esp_now_send_info_t *tx_info,
                   esp_now_send_status_t status) {
  // called when packet finished sending
}

void connection_init() {
  esp_netif_init();
  esp_event_loop_create_default();

  wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&wifi_config);

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();
  esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  esp_now_init();
  esp_now_register_send_cb(send_callback);
  esp_now_register_recv_cb(receive_callback);

  // on connection
  esp_now_add_peer(const esp_now_peer_info_t *peer);
  esp_now_set_peer_rate_config(const uint8_t *peer_addr,
                               esp_now_rate_config_t *config);
}

void connection_deinit() {
  esp_now_deinit();
  // TODO: deinit wifi
}

void setup() { connection_init(); }

void loop() {
  if (peer_connected) {
    const uint8_t data[1] = 7;
    esp_now_send(peer_addr, data, sizeof data);
  }
}
