#pragma once
#include "FreeRTOS.h"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>

struct HostQueue {
  size_t capacity, item_size;
  std::deque<std::vector<uint8_t>> items;
};
inline QueueHandle_t xQueueCreate(size_t capacity, size_t item_size) {
  return new HostQueue{capacity, item_size, {}};
}
inline void vQueueDelete(QueueHandle_t queue) { delete static_cast<HostQueue *>(queue); }
inline BaseType_t xQueueSend(QueueHandle_t queue, const void *data, unsigned) {
  auto &q = *static_cast<HostQueue *>(queue);
  if (q.items.size() == q.capacity)
    return pdFALSE;
  const auto *bytes = static_cast<const uint8_t *>(data);
  q.items.emplace_back(bytes, bytes + q.item_size);
  return pdTRUE;
}
inline BaseType_t xQueueReceive(QueueHandle_t queue, void *data, unsigned) {
  auto &q = *static_cast<HostQueue *>(queue);
  if (q.items.empty())
    return pdFALSE;
  std::memcpy(data, q.items.front().data(), q.item_size);
  q.items.pop_front();
  return pdTRUE;
}
inline UBaseType_t uxQueueMessagesWaiting(QueueHandle_t queue) {
  return static_cast<HostQueue *>(queue)->items.size();
}
inline void xQueueReset(QueueHandle_t queue) {
  static_cast<HostQueue *>(queue)->items.clear();
}
