#include <esp_wifi.h>
#include <esp_now.h>

#define MAC_LEN 6

const uint8_t BROADCAST_ADDR[MAC_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, BROADCAST_ADDR, ESP_NOW_ETH_ALEN) == 0)

const uint8_t CONFIG_ESPNOW_CHANNEL = 6;

uint8_t peer_addr[MAC_LEN];
bool peer_connected = false;

int rssi_packet_count = 0;

void add_peer(const uint8_t *peer_addr) {
  esp_now_peer_info_t broadcast_peer{0};
  broadcast_peer.channel = CONFIG_ESPNOW_CHANNEL;
  broadcast_peer.ifidx = WIFI_IF_STA;
  broadcast_peer.encrypt = false;
  memcpy(broadcast_peer.peer_addr, peer_addr, MAC_LEN);

  ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));
}

void receive_callback(const esp_now_recv_info_t *rx_info, const uint8_t *data,
                      int data_len) {
  if (rx_info->rx_ctrl) {
    rssi_packet_count++;
    // rx_info->rx_ctrl->rssi;
  }
  // rx_info->rx_ctrl->rssi;
  // rx_info->rx_ctrl->timestamp;
  // TODO: what about rx_info->rx_ctrl->noise_floor?

  // TODO: move this logic to not be inside callback?

  if (IS_BROADCAST_ADDR(rx_info->des_addr)) {
    // regardless of broadcast/unicast, as slave add as peer
    if (!esp_now_is_peer_exist(rx_info->src_addr)) {
      add_peer(rx_info->src_addr);
      memcpy(peer_addr, rx_info->src_addr, MAC_LEN);
      peer_connected = true;
    }
  }
}

void send_callback(const esp_now_send_info_t *tx_info,
                   esp_now_send_status_t status) {
  // called when packet finished sending
}

void connection_init() {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(
      esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

  // TODO: deinit stuff on any errors?

  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_send_cb(send_callback));
  ESP_ERROR_CHECK(esp_now_register_recv_cb(receive_callback));

  // add the broadcast addr to send broadcast data
  add_peer(BROADCAST_ADDR);

  // set high speed for broadcast only
  esp_now_rate_config_t tx_rate = {WIFI_PHY_MODE_11G, WIFI_PHY_RATE_6M,
                                         false, false};

  // const esp_now_rate_config_t tx_rate = {WIFI_PHY_MODE_11G,
  // WIFI_PHY_RATE_24M,
  //                                        false, false};

  ESP_ERROR_CHECK(esp_now_set_peer_rate_config(BROADCAST_ADDR, &tx_rate));
}

void setup() {
  Serial.begin();
  connection_init();
}

// int packet_count = 0;
// int packet_yield_count = 3000;

uint32_t last_print_timestamp = 0;

void loop() {
  if (peer_connected) {
    // send actual information over unicast information
    const uint8_t data[1] = {155};
    esp_now_send(peer_addr, data, sizeof data);
  }

  // constantly send rssi packets
  // const uint8_t rssi_data[1] = 7;
  // esp_err_t err = esp_now_send(BROADCAST_ADDR, rssi_data, sizeof rssi_data);
  //
  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "RSSI packet send failure: %d", err);
  // }

  // print packet count every half second
  if (millis() - last_print_timestamp > 500) {
    Serial.printf("rssi_packet_count: %d\n", rssi_packet_count);
    rssi_packet_count = 0;
    last_print_timestamp = millis();
  }
  vTaskDelay(1);

  // packet_count++;

  // // must delay every so often to make watchdog happy
  // if (packet_count > packet_yield_count) {
  //   packet_count = 0;
  //   vTaskDelay(1);
  // }
}
