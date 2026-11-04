#include <arx_lift_src/upward_damping.hpp>
#include <cmath>
#include <iostream>
#include <limits>
int main() {
 using namespace arx_lift_experiment;
 if (!(apply(-2.8, -.5, true, -17, 13)>-2.8)) { std::cerr<<"FAIL: upward braking term missing\n";return 1; }
 for (int flags=0;flags<16;++flags) {
  bool gate=active(flags&1,flags&2,flags&4,flags&8);
  if (gate!=(flags==15)) return 2;
 }
 int checks=0;
 for (double base : {-17.,-3.,-2.,0.,13.}) for (int i=-450;i<=450;++i) {
  double v=i/10.;double result=apply(base,v,true,-17,13);double delta=result-base;
  if (std::abs(delta)>.100000001 || delta*v>1e-10 || result< -17 || result>13) return 3;
  if (apply(base,v,false,-17,13)!=base) return 4;
  if (std::abs(v)<=.05 && result!=base) return 5;
  ++checks;
 }
 for (double v : {std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) if(apply(-2.8,v,true,-17,13)!=-2.8) return 6;
 std::cout<<"PASS: "<<checks<<" bound/passivity cases; disabled, hold, homing and down gates verified\n";
}
