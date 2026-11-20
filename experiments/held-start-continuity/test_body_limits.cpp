#include "slow_lift_config.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
  assert(valid_lift_target(0) && valid_lift_target(18));
  for(double value : {-0.001,18.001,std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) assert(!valid_lift_target(value));
  assert(valid_chassis_target(.05,0,0,1));
  assert(valid_chassis_target(0,-.05,0,1));
  assert(valid_chassis_target(0,0,-.1,1));
  assert(valid_chassis_target(.8,0,0,1));
  assert(valid_chassis_target(0,-.8,0,1));
  assert(valid_chassis_target(0,0,-.4,1));
  assert(!valid_chassis_target(.8001,0,0,1));
  assert(!valid_chassis_target(.8,.8,0,1));
  assert(!valid_chassis_target(0,0,.4001,1));
  assert(valid_chassis_target(0,0,0,2));
  assert(!valid_chassis_target(.01,0,0,2));
  assert(!valid_chassis_target(0,0,0,3));
  assert(!valid_chassis_target(std::numeric_limits<double>::quiet_NaN(),0,0,1));
  assert(!chassis_expired(.249) && chassis_expired(.25) && chassis_expired(1));
  std::cout << "PASS: lift 0..18, bounded chassis, stop mode, unconditional 250ms timeout\n";
}
