#include "chassis_motion.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using Velocity = ChassisMotion::Velocity;
int main() {
  ChassisMotion quick_turn;
  quick_turn.step(10, 100, true);
  for (int i = 1; i <= 100; ++i) {
    const double tick = 10 + i * .0025;
    quick_turn.request({0, 0, WEB_MAX_YAW}, 1, tick);
    quick_turn.step(tick, 0, true);
  }
  assert(std::abs(quick_turn.velocity()[2] - WEB_MAX_YAW) < 1e-12);
  assert(quick_turn.velocity()[0] == 0 && quick_turn.velocity()[1] == 0);

  ChassisMotion motion;
  double now = 1;
  motion.step(now, 100, true);
  assert(motion.mode() == 2 && motion.velocity() == Velocity{});

  // Full diagonal speed and yaw stay inside the existing limits and rate bounds.
  const double diagonal = WEB_MAX_LINEAR / std::sqrt(2.0);
  const Velocity forward{diagonal, diagonal, WEB_MAX_YAW};
  for (int i = 0; i < 500; ++i) {
    const Velocity before = motion.velocity();
    motion.request(forward, 1, now);
    now += .0025;
    motion.step(now, .0025, true);
    const auto &v = motion.velocity();
    assert(std::hypot(v[0], v[1]) <= WEB_MAX_LINEAR + 1e-12);
    assert(std::hypot(v[0] - before[0], v[1] - before[1]) <= WEB_LINEAR_ACCEL * .0025 + 1e-12);
    assert(std::abs(v[2] - before[2]) <= WEB_YAW_ACCEL * .0025 + 1e-12);
  }
  assert(motion.velocity() == forward);

  // All axes pass through zero and hold before reversing.
  const Velocity reverse{-diagonal, -diagonal, -WEB_MAX_YAW};
  double first_zero = -1, first_reverse = -1;
  for (int i = 0; i < 850; ++i) {
    const Velocity before = motion.velocity();
    motion.request(reverse, 1, now);
    now += .0025;
    motion.step(now, .0025, true);
    const auto &v = motion.velocity();
    assert(std::hypot(v[0] - before[0], v[1] - before[1]) <= WEB_LINEAR_DECEL * .0025 + 1e-12);
    assert(std::abs(v[2] - before[2]) <= std::max(WEB_YAW_ACCEL, WEB_YAW_DECEL) * .0025 + 1e-12);
    if (v == Velocity{} && first_zero < 0) first_zero = now;
    if (v[0] < 0 && first_reverse < 0) first_reverse = now;
  }
  assert(first_zero > 0 && first_reverse - first_zero >= WEB_REVERSE_HOLD_S);
  assert(motion.velocity() == reverse);

  // Space requests zero speed immediately, holds it, and cannot be bypassed.
  const double stopped = now;
  motion.stop(now);
  assert(motion.velocity() == Velocity{} && motion.mode() == 1);
  for (int i = 0; i < 100; ++i) {
    now += .005;
    motion.request(forward, 1, now);
    motion.step(now, .005, true);
    assert(motion.velocity() == Velocity{} && motion.mode() == 1);
  }
  motion.stop(now);  // Repeated stop packets must not extend the brake forever.
  now = stopped + WEB_BRAKE_HOLD_S + .001;
  motion.step(now, 10, true);
  assert(motion.mode() == 2 && motion.velocity() == Velocity{});

  motion.request(forward, 1, now);
  now += .01;
  motion.step(now, .01, true);
  assert(motion.velocity()[0] > 0);
  now += WEB_CHASSIS_TIMEOUT;
  motion.step(now, WEB_CHASSIS_TIMEOUT, true);
  assert(motion.velocity() == Velocity{} && motion.mode() == 1);
  now += WEB_BRAKE_HOLD_S + .001;
  motion.step(now, 5, true);
  assert(motion.mode() == 2);

  // Faults and lost calibration keep the existing protection path.
  motion.request(forward, 1, now);
  motion.step(now + .01, 0, false);
  assert(motion.mode() == 2 && motion.velocity() == Velocity{});
  motion.request({.8, .8, 0}, 1, now);
  assert(motion.mode() == 2);
  motion.request(forward, 1, now);
  motion.step(std::numeric_limits<double>::quiet_NaN(), 0, true);
  assert(motion.mode() == 2 && motion.velocity() == Velocity{});
  std::cout << "PASS: combined slew limits, reverse zero hold, bounded zero-speed brake, watchdog and fault protection\n";
}
