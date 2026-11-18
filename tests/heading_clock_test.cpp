#include "heading_clock.h"
#include <cassert>
#include <cmath>

int main() {
  constexpr float pi = 3.14159265358979323846f;
  HeadingClock clock;
  const uint32_t origin = UINT32_MAX - 5000;
  clock.publish(origin, 0, 1500);
  assert(std::abs(clock.render(origin + 10000, 0) - pi / 2) < .0001f);
  // A new RPM changes the derivative, not the already accumulated phase.
  clock.publish(origin + 10000, pi / 2, 3000);
  assert(std::abs(clock.render(origin + 10000, 0) - pi / 2) < .0001f);
  assert(std::abs(clock.render(origin + 15000, 0) - pi) < .0001f);

  HeadingClock turn;
  turn.publish(0, 0, 0);
  turn.render(0, 1);
  for (unsigned t = 1000; t <= 20000; t += 1000) turn.render(t, 1);
  const float before = turn.angle();
  // Physical phase corrections do not overwrite commanded heading offset.
  turn.publish(20000, .4f, 0);
  assert(std::abs(turn.render(20000, 0) - (.4f - .02f)) < .0001f);
  assert(before > 6); // Wrapped negative heading offset.
  // Turn control integrates elapsed time, independently of fast-loop cadence.
  HeadingClock slow;
  slow.publish(0, 0, 0);
  slow.render(0, 1);
  for (unsigned t = 5000; t <= 20000; t += 5000) slow.render(t, 1);
  assert(std::abs(slow.angle() - before) < .0001f);

  // No unbounded prediction or retroactive turning across a blocked task.
  clock.publish(1000, 0, 1500);
  const float expired = clock.render(1001000, 0);
  assert(std::abs(clock.render(2001000, 0) - expired) < .0001f);
  HeadingClock invalid;
  invalid.publish(0, NAN, 1500);
  assert(std::isfinite(invalid.render(1000, 0)));
}
