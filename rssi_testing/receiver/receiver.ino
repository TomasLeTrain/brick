#include <espnow.h>

// TODO: send empty broadcast packets for rssi information

void connection_init(){
  // TODO: start wifi
  esp_now_init()
}

void connection_deinit(){
  esp_now_deinit()
  // TODO: deinit wifi
}

void setup() {
  connection_init()

}

void loop() {
  esp_err_t res = esp_now_send(const uint8_t *peer_addr, const uint8_t *data, size_t len);
}

