#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>
using rmt_channel_handle_t = void *;
using rmt_encoder_handle_t = void *;
constexpr int ESP_OK = 0, ESP_ERR_TIMEOUT = 1, RMT_CLK_SRC_DEFAULT = 0;
struct rmt_symbol_word_t {
  uint16_t duration0, duration1;
  uint8_t level0, level1;
};
struct rmt_tx_channel_config_t {
  int gpio_num, clk_src;
  uint32_t resolution_hz;
  size_t mem_block_symbols, trans_queue_depth;
};
struct rmt_copy_encoder_config_t {};
struct rmt_transmit_config_t {};
struct HostRmtChannel {
  bool busy = false;
  const void *payload = nullptr;
  size_t size = 0;
  int pin = 0;
  size_t symbols = 0;
};
inline std::vector<HostRmtChannel *> host_rmt_channels;
inline void host_rmt_complete_all() {
  for (auto *c : host_rmt_channels)
    c->busy = false;
}
inline const void *host_rmt_payload = nullptr;
inline size_t host_rmt_size = 0;
inline unsigned host_rmt_transmits = 0;
inline int rmt_new_tx_channel(const rmt_tx_channel_config_t *config,
                              rmt_channel_handle_t *out) {
  auto *channel = new HostRmtChannel;
  channel->pin = config->gpio_num;
  channel->symbols = config->mem_block_symbols;
  host_rmt_channels.push_back(channel);
  *out = channel;
  return ESP_OK;
}
inline int rmt_new_copy_encoder(const rmt_copy_encoder_config_t *,
                                rmt_encoder_handle_t *out) {
  *out = &host_rmt_payload;
  return ESP_OK;
}
inline int rmt_enable(rmt_channel_handle_t) { return ESP_OK; }
inline int rmt_disable(rmt_channel_handle_t) { return ESP_OK; }
inline int rmt_del_channel(rmt_channel_handle_t handle) {
  auto *channel = static_cast<HostRmtChannel *>(handle);
  host_rmt_channels.erase(
      std::remove(host_rmt_channels.begin(), host_rmt_channels.end(), channel),
      host_rmt_channels.end());
  delete channel;
  return ESP_OK;
}
inline int rmt_del_encoder(rmt_encoder_handle_t) { return ESP_OK; }
inline int rmt_tx_wait_all_done(rmt_channel_handle_t handle, int) {
  return static_cast<HostRmtChannel *>(handle)->busy ? ESP_ERR_TIMEOUT : ESP_OK;
}
inline int rmt_transmit(rmt_channel_handle_t handle, rmt_encoder_handle_t,
                        const void *data, size_t size,
                        const rmt_transmit_config_t *) {
  host_rmt_payload = data;
  host_rmt_size = size;
  auto *channel = static_cast<HostRmtChannel *>(handle);
  channel->busy = true;
  channel->payload = data;
  channel->size = size;
  ++host_rmt_transmits;
  return ESP_OK;
}
