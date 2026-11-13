// Pure offline replay: no ROS, CAN, HTTP, or motor interface.
// Compile against the exact hardware headers using -I/path/to/web/hardware.
#include "chassis_motion.hpp"
#include <iomanip>
#include <iostream>
#include <vector>

struct Sample { double t; ChassisMotion::Velocity v; };

int main() {
  std::vector<Sample> samples;
  Sample row;
  while (std::cin >> row.t >> row.v[0] >> row.v[1] >> row.v[2])
    samples.push_back(row);
  if (samples.empty()) return 2;
  std::cout << std::setprecision(12);
  for (int variant = 0; variant < 4; ++variant) {
    ChassisMotion motion(2.0, 2.0);
    motion.step(10, 100, true);
    std::size_t next = 0;
    double last = -100, peak = 0, integral = 0, responsive = 0;
    double zero = 0, below = 0, active = 0;
    const double end = samples.back().t;
    for (double t = .0025; t <= end; t += .0025) {
      while (next < samples.size() && samples[next].t <= t) {
        auto target = samples[next].v;
        if (variant == 1 || variant == 3) target[2] = 0;
        if (variant == 2 || variant == 3) target[1] = 0;
        motion.request(target, 1, 10 + samples[next].t);
        last = samples[next].t;
        ++next;
      }
      motion.step(10 + t, t - last, true);
      const double backward = -motion.velocity()[0];
      peak = std::max(peak, backward);
      integral += backward * .0025;
      if (backward >= .14) responsive += .0025;
      if (next) {
        active += .0025;
        if (backward < 1e-9) zero += .0025;
        if (backward < .14) below += .0025;
      }
    }
    std::cout << variant << ' ' << peak << ' ' << integral / end << ' '
              << responsive << ' ' << zero << ' ' << below << ' ' << active << '\n';
  }
}
