#pragma once
#include <cstdint>

// Set BOTH ESPs to the same channel before an event. Do not scan/hop while
// armed.
constexpr uint8_t RADIO_CHANNEL = 6;
static_assert(RADIO_CHANNEL >= 1 && RADIO_CHANNEL <= 11, "Use channel 1..11");
// Optional fixed peer MACs prevent accidental pairing with another team's
// NowPair devices. All zero means discovery. These filters are not encryption.
constexpr uint8_t ROBOT_MAC[6] = {0, 0, 0, 0, 0, 0};
constexpr uint8_t SENDER_MAC[6] = {0, 0, 0, 0, 0, 0};
