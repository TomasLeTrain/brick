#pragma once
constexpr int WIFI_STA = 1;
struct HostWiFi {
  bool mode(int) { return true; }
  bool setSleep(bool) { return true; }
};
inline HostWiFi WiFi;
