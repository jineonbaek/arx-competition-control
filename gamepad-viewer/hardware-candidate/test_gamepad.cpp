#include "chassis_motion.hpp"
#include "command_lease.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
int main(){
  assert(!valid_chassis_target(2,0,0,1));
  assert(valid_chassis_target(2,0,2,1,2,2));
  assert(!valid_chassis_target(2,2,0,1,2,2));
  ChassisMotion gamepad(2,2);gamepad.step(1,0,true);
  for(int i=1;i<=1600;++i){double t=1+i*.0025;gamepad.request({2,0,2},1,t);gamepad.step(t,0,true);}
  assert(std::abs(gamepad.velocity()[0]-2)<1e-9);
  assert(std::abs(gamepad.velocity()[2]-2)<1e-9);
  gamepad.step(5.01,.25,true);assert(gamepad.velocity()==ChassisMotion::Velocity{});
  CommandLease lease;
  assert(lease.accept(CommandSource::Web,1,1));
  assert(!lease.accept(CommandSource::Gamepad,1,1.1));
  assert(!lease.accept(CommandSource::Gamepad,2,1.1));
  assert(lease.accept(CommandSource::Gamepad,1,1.3));
  assert(!lease.accept(CommandSource::Web,1,1.4));
  assert(lease.accept(CommandSource::Web,2,1.4));
  assert(!lease.accept(CommandSource::Gamepad,1,1.5));
  assert(lease.accept(CommandSource::Gamepad,1,1.7));
  assert(!lease.accept(CommandSource::Gamepad,1,NAN));
  std::cout<<"PASS: SDK 2 target, legacy limits, watchdog, source arbitration\n";
}
