#pragma once
#include "Arduino.h"
constexpr int ESP_PARTITION_TYPE_DATA = 1, ESP_PARTITION_SUBTYPE_ANY = 255;
inline const void *esp_partition_find_first(int, int, const char *) {
  flash_operation();
  static int partition;
  return &partition;
}
