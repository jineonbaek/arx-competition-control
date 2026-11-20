#include "chassis_output.hpp"
#include <cassert>
#include <iostream>
#include <limits>
int main() {
  ChassisOutputGate gate("driver-1");
  assert(gate.bind_scheduler("scheduler-1"));
  ChassisOutput a{"driver-1",10,100.,.3,{.2,0.,.1}};
  a.scheduler_instance="scheduler-1";
  assert(gate.accept(a,100.01,true));
  auto b=a;b.seq=12;b.sdk={0.,.1,0.};assert(gate.accept(b,100.02,true));
  assert(!gate.accept(a,100.03,true)); // delayed earlier writer
  b.seq=13;b.instance="old";assert(!gate.accept(b,100.03,true));
  b.instance="driver-1";assert(!gate.accept(b,100.31,true));
  assert(!gate.accept(b,99.,true));
  b.issued=100.03;b.sdk[0]=std::numeric_limits<double>::quiet_NaN();
  assert(!gate.accept(b,100.04,true));assert(gate.sequence()==12);
  b.sdk={.2,0.,0.};assert(!gate.accept(b,100.04,false));
  b.sdk={0.,0.,0.};assert(gate.accept(b,100.04,false));
  b.seq=14;b.sdk={.2,0.,0.};assert(gate.accept(b,100.04,true));
  ChassisOutputGate restarted("driver-2");assert(!restarted.accept(b,100.04,true));
  b.instance="driver-2";assert(!restarted.accept(b,100.04,true));
  assert(restarted.bind_scheduler("scheduler-2"));
  assert(!restarted.accept(b,100.04,true));
  b.scheduler_instance="scheduler-2";assert(restarted.accept(b,100.04,true));
  std::cout<<"PASS: ordered SDK output, restart fencing, expiry, invalid requests do not stop\n";
}
