#include "start_reference.hpp"
#include <cassert>
#include <cmath>
int main() {
  std::vector<double> actual(7,0.), held(6,0.), first(6,0.);
  held[3]=.012;first=held;
  auto continuous=choose_start_reference(held,actual,first);
  assert(continuous.preserved && continuous.value==held);
  first[3]=0.;
  auto ordinary=choose_start_reference(held,actual,first);
  assert(!ordinary.preserved && ordinary.value[3]==0.);
  held[3]=.1;first=held;
  auto too_far=choose_start_reference(held,actual,first);
  assert(!too_far.preserved && too_far.value[3]==0.);
  first[3]=NAN;
  bool rejected=false;try{choose_start_reference(held,actual,first);}catch(...){rejected=true;}
  assert(rejected);
  assert(choose_start_gripper_target(-.81,-3.14,0.)==-.81);
  assert(std::isnan(choose_start_gripper_target(1.,-3.14,0.)));
  rejected=false;try{choose_start_gripper_target(-4.,-3.14,0.);}catch(...){rejected=true;}
  assert(rejected);
}
