#include "arm_limits.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
  {
    ArmLimits caps{{{{-90,90},{-90,90},{-90,90},{-90,90},{-90,90},{-90,90}}},{{-3.14,0}},10};
    std::vector<double> q(6,0),actual(7,0),target{.1,.2,.3,.4,.5,.6};
    std::array<double,6> speeds{15,15,15,50,50,50};
    auto next=caps.advance(q,target,actual,speeds,.01);
    for(int j=0;j<6;++j)assert(std::abs(next[j])*180/M_PI<=speeds[j]*.01+1e-10);
    assert(std::abs(next[2]*180/M_PI-.15)<1e-10);
    target={0,0,0,.1,.2,.3};next=caps.advance(q,target,actual,speeds,.01);
    assert(std::abs(next[5]*180/M_PI-.5)<1e-10);
  }

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
  assert(gripper_hold_target(limits,-6.305,0)==0);
  assert(gripper_hold_target(limits,0,-3.14)==-3.14);
  assert(gripper_hold_target(limits,-6.305,NAN)==-3.14);
  // Completed loaded reference must not be rebased to sagged measurements.
  const std::vector<double> sagged{.1,.2,.3,.4,.5,.6,-.86};
  const std::vector<double> issued{.1,.2,.31,.41,.51,.62};
  const std::vector<double> measured{.1,.2,.3,.4,.5,.6};
  assert(arm_stop_target(limits,sagged,issued,true)==issued);
  assert(arm_stop_target(limits,sagged,issued,false)==measured);
  auto bad=issued;bad[5]=.64;
  assert(arm_stop_target(limits,sagged,bad,true)==measured);
  bad=issued;bad[3]=NAN;
  assert(arm_stop_target(limits,sagged,bad,true)==measured);
  bad=issued;bad.pop_back();
  assert(arm_stop_target(limits,sagged,bad,true)==measured);
  bad=issued;bad[0]=2.;
  assert(arm_stop_target(limits,sagged,bad,true)==measured);
  bad=issued;bad[5]=.6+2.*M_PI/180.;
  assert(arm_stop_target(limits,sagged,bad,true)==bad);
  bad[5]+=1e-6;
  assert(arm_stop_target(limits,sagged,bad,true)==measured);
  assert(completed_hold_allowed("manual_stop","reference_complete",5,true));
  assert(!completed_hold_allowed("manual_stop","running",5,true));
  assert(!completed_hold_allowed("tracking_error","reference_complete",5,true));
  assert(!completed_hold_allowed("manual_stop","reference_complete",2,true));
  // An old completed trajectory must not qualify later manual references.
  assert(!completed_hold_allowed("manual_stop","reference_complete",5,false));
  auto near_limit=sagged;near_limit[0]=89.*M_PI/180.;
  bad=measured;bad[0]=90.*M_PI/180.;
  assert(arm_stop_target(limits,near_limit,bad,true)[0]==near_limit[0]);
  bool kept=true;
  auto fallback=arm_stop_target(limits,near_limit,bad,true,&kept);
  assert(!kept);
  auto drifted=near_limit;drifted[3]+=.005;
  auto repeated=arm_stop_target(limits,drifted,fallback,
      completed_hold_allowed("manual_stop","reference_complete",5,kept),&kept);
  assert(!kept && repeated[3]==drifted[3]);
  assert(std::isnan(startup_gripper_target(limits,NAN)));
  assert(startup_gripper_target(limits,0.)==0.);
  bool rejected=false;try{startup_gripper_target(limits,.1);}catch(const std::invalid_argument&){rejected=true;}
  assert(rejected);
  rejected=false;try{startup_gripper_target(limits,INFINITY);}catch(const std::invalid_argument&){rejected=true;}
  assert(rejected);
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
