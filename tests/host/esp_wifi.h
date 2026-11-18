#pragma once
constexpr int WIFI_SECOND_CHAN_NONE = 0, ESP_OK = 0;
inline int esp_wifi_set_channel(unsigned, int) { return ESP_OK; }
