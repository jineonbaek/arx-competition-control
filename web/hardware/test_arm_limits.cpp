#include "arm_limits.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
  ArmLimits limits{{{{-89.9,89.9},{-5.7,206.2},{-5.7,171.8},{-73.9,73.9},{-84.7,84.7},{-99.6,99.6}}},{-3.14,0},5.7};
  std::vector<double> actual(7,0), target(6,0);
  assert(limits.valid(target,-1,actual,5));
  for(int i=0;i<6;++i) {
    target.assign(6,0);target[i]=5.7*M_PI/180;assert(limits.valid(target,-3.14,actual,5));
    target[i]+=0.0001;assert(limits.valid(target,0,actual,5));
    target[i]=limits.degrees[i][1]*M_PI/180;actual[i]=target[i];assert(limits.valid(target,0,actual,5));
    target[i]+=0.0001;assert(!limits.valid(target,0,actual,5));actual.assign(7,0);
  }
  target.assign(6,0);target[0]=45*M_PI/180;
  assert(limits.valid(target,0,actual,5));
  std::vector<double> reference(6,0);
  auto next=limits.advance(reference,target,actual,5.7,.01);
  assert(next[0]>0 && next[0]<=5.7*M_PI/180*.01+1e-12);
  reference[0]=5.7*M_PI/180;next=limits.advance(reference,target,actual,5.7,.01);
  assert(next==reference);
  reference.assign(6,0);target[0]=.0001;next=limits.advance(reference,target,actual,5.7,.01);
  assert(next==target);
  assert(limits.valid_gripper(0) && limits.valid_gripper(-3.14));
  assert(!limits.valid_gripper(.001) && !limits.valid_gripper(-3.141));
  target.assign(6,0);
  for(int mode=0;mode<4;++mode) assert(limits.valid(target,-1,actual,mode));
  assert(!limits.valid(target,-1,actual,4));assert(!limits.valid(target,.1,actual,5));
  target[0]=std::numeric_limits<double>::quiet_NaN();assert(!limits.valid(target,0,actual,5));
  assert(arm_feedback_index(1,8,0,false)==0 && arm_feedback_index(4,8,0,false)==2);
  for(unsigned id=5;id<=8;++id) assert(arm_feedback_index(0,8,id,false)==int(id)-2);
  assert(arm_feedback_index(1,8,0,true)==-1 && arm_feedback_index(0,8,8,true)==-1);
  assert(arm_feedback_index(0,8,1,false)==-1 && arm_feedback_index(0x7ff,8,5,false)==-1);
  std::cout<<"PASS: joint bounds, delta, gripper, modes, actual CAN replies only\n";
}
