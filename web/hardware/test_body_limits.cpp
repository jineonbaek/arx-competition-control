#include "slow_lift_config.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
  struct Config { double lift_max_vel=7.; double lift_calibrate_vel=8.;
                  double lift_kp=2.; double gravity_compensation_torque=-2.; };
  Config config;
  apply_slow_lift_config(config);
  assert(std::abs(config.lift_max_vel*400.-1.)<1e-12);
  assert(config.lift_calibrate_vel==.5);
  apply_slow_lift_config(config,.5);
  assert(std::abs(config.lift_max_vel*400.-.5)<1e-12);
  assert(config.lift_calibrate_vel==.5);
  assert(config.lift_kp==2. && config.gravity_compensation_torque==-2.);
  for(double rate : {0.,-.1,1.0001,std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::infinity()}) {
    Config unchanged;
    bool rejected=false;
    try { apply_slow_lift_config(unchanged,rate); }
    catch(const std::invalid_argument&) { rejected=true; }
    assert(rejected);
    assert(unchanged.lift_max_vel==7. && unchanged.lift_calibrate_vel==8.);
  }
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
  std::cout << "PASS: bounded lift ramp and height, bounded chassis, stop mode, unconditional 250ms timeout\n";
}
