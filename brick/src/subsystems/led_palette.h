#pragma once
#include <cmath>
#include <cstdint>

struct LedColor {
  uint8_t red, green, blue;
};

namespace LedPalette {
constexpr LedColor background{255, 255, 255};
constexpr LedColor text{0, 0, 255};
constexpr LedColor vector{255, 0, 128};

// An involution: applying this twice restores every color. Red/green reversal
// swaps white and blue, and pink and mint. The dedicated wheel pixel uses a
// full RGB complement instead, preserving saturation at the opposite hue.
constexpr LedColor flip(LedColor color, bool wheel = false) {
  return {uint8_t(255 - color.red), uint8_t(255 - color.green),
          wheel ? uint8_t(255 - color.blue) : color.blue};
}

inline LedColor hue(float turns) {
  if (!std::isfinite(turns))
    return {0, 0, 0};
  turns -= std::floor(turns);
  const float sector = turns * 6;
  const int index = int(sector);
  const uint8_t up = uint8_t(std::lround((sector - index) * 255));
  const uint8_t down = 255 - up;
  switch (index) {
  case 0:
    return {255, up, 0};
  case 1:
    return {down, 255, 0};
  case 2:
    return {0, 255, up};
  case 3:
    return {0, down, 255};
  case 4:
    return {up, 0, 255};
  default:
    return {255, 0, down};
  }
}
} // namespace LedPalette
