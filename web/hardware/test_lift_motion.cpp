#include "lift_motion.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

bool near(double a,double b,double tol=1e-8){return std::abs(a-b)<=tol;}
LiftState tick(LiftMotion& m,double& sdk,double dt,bool calibrated=true){
  auto s=m.advance(sdk,dt,calibrated);
  const double cap=m.sdk_step_limit(dt);
  sdk+=std::clamp(s.position-sdk,-cap,cap);
  assert(near(sdk,s.position));
  assert(s.position>=-1e-9 && s.position<=18.+1e-9);
  assert(std::abs(s.velocity)<=2.+1e-8);
  assert(std::abs(s.acceleration)<=4.+1e-8);
  assert(std::abs(s.jerk)<=16.+1e-8);
  return s;
}
void finish(LiftMotion& m,double& sdk,double target){
  for(int i=0;i<30000 && m.active();++i)tick(m,sdk,.0025);
  assert(!m.active() && m.fault().empty());
  assert(near(m.state().position,target));
  assert(near(m.state().velocity,0.) && near(m.state().acceleration,0.));
}
int main(){
  {
    LiftMotion m;
    m.advance(.4,.0025,false);
    assert(near(m.target(),0.));  // Boot target survives homing's changing reference.
    m.advance(.2,.0025,false);
    double sdk=.1;auto s=tick(m,sdk,.1,true);
    assert(near(s.position,.1) && near(s.velocity,0.) && m.fault().empty());
    finish(m,sdk,0.);
  }
  for(const auto& limits:std::vector<std::vector<double>>{{2.001,4.,16.},{2.,4.001,16.},{2.,4.,16.001},{0.,4.,16.}}){
    bool rejected=false;try{LiftMotion m(limits[0],limits[1],limits[2]);}
    catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  {
    LiftMotion m;double sdk=0.;m.hold(sdk);m.request(8.);tick(m,sdk,0.);
    // V=2, A=4, J=16: two 0.75s ramps plus 3.25s cruise.
    assert(near(m.duration(),4.75));
    LiftState previous=m.state();double highest_velocity=0.;
    for(int i=0;i<4000 && m.active();++i){
      const auto s=tick(m,sdk,.0025);
      assert(s.position>=previous.position-1e-9);
      assert(std::abs(s.acceleration-previous.acceleration)<=16.*.0025+1e-8);
      assert(std::abs(s.velocity-previous.velocity)<=4.*.0025+1e-8);
      highest_velocity=std::max(highest_velocity,s.velocity);previous=s;
    }
    assert(highest_velocity>1.99 && near(sdk,8.));
    m.request(0.);finish(m,sdk,0.);
  }
  {
    LiftMotion m;double sdk=0.;m.hold(sdk);m.request(.02);
    double peak=0.;
    for(int i=0;i<3000 && m.active();++i){auto s=tick(m,sdk,.001);peak=std::max(peak,s.velocity);}
    assert(near(sdk,.02) && peak>0. && peak<2. && !m.active());
  }
  {
    LiftMotion m;double sdk=0.;m.hold(sdk);m.request(8.);
    for(int i=0;i<120;++i)tick(m,sdk,.0025);
    auto before=m.state();m.request(0.);auto after=tick(m,sdk,0.);
    assert(near(before.position,after.position));
    assert(near(before.velocity,after.velocity));
    assert(near(before.acceleration,after.acceleration));
    finish(m,sdk,0.);
  }
  {
    LiftMotion m;double sdk=0.;m.hold(sdk);m.request(8.);
    for(int i=0;i<500;++i){m.request(8.);tick(m,sdk,.0025);}
    assert(m.plan_count()==1);finish(m,sdk,8.);
  }
  {
    LiftMotion m;double sdk=0.;m.hold(sdk);m.request(6.);
    const std::vector<double> dt{.001,.004,.002,.0035,.0015};
    for(int i=0;i<10000 && m.active();++i)tick(m,sdk,dt[i%dt.size()]);
    assert(near(sdk,6.) && m.fault().empty());
  }
  {
    LiftMotion m;double sdk=3.;m.hold(sdk);m.request(8.);
    for(int i=0;i<150;++i)tick(m,sdk,.0025);
    m.hold(sdk);assert(!m.active());
    for(int i=0;i<30;++i)tick(m,sdk,.0025);
    assert(near(m.target(),sdk) && near(m.state().velocity,0.));
  }
  {
    LiftMotion m;double sdk=0.;m.hold(sdk);m.request(8.);tick(m,sdk,.0025);
    tick(m,sdk,.1);assert(!m.active() && !m.fault().empty());
    const double held=sdk;
    for(int i=0;i<20;++i)tick(m,sdk,.0025);
    assert(near(sdk,held));
    m.request(8.);finish(m,sdk,8.);
  }
  {
    LiftMotion m;double sdk=0.;m.hold(sdk);m.request(8.);
    for(int i=0;i<100;++i)tick(m,sdk,.0025);
    tick(m,sdk,.0025,false);
    assert(!m.active());double held=sdk;
    for(int i=0;i<20;++i)tick(m,sdk,.0025,true);
    assert(near(sdk,held));m.request(8.);finish(m,sdk,8.);
  }
  {
    LiftMotion m;double sdk=0.;m.hold(sdk);m.request(8.);tick(m,sdk,.0025);
    sdk+=.01;tick(m,sdk,.0025);
    assert(!m.active() && !m.fault().empty() && near(m.target(),sdk));
  }
  for(double t:{0.,.05,.30,.75,2.,4.,8.5,9.7}){
    LiftMotion m;double sdk=0.;m.hold(sdk);m.request(18.);tick(m,sdk,0.);
    for(double e=0;e<t;e+=.0025)tick(m,sdk,.0025);
    m.request(0.);finish(m,sdk,0.);
  }
  {
    LiftMotion m;double sdk=0.;m.hold(sdk);m.request(8.);
    for(double v:{-1.,18.001,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}){
      bool rejected=false;try{m.request(v);}catch(const std::invalid_argument&){rejected=true;}
      assert(rejected && m.target()==8.);
    }
    finish(m,sdk,8.);
    for(double dt:{-.001,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}){
      m.request(0.);tick(m,sdk,dt);
      assert(!m.active() && !m.fault().empty());
    }
  }
  std::cout<<"PASS: jerk-limited lift bounds, reversal continuity, short moves, duplicate targets, timing, stop and calibration cancellation\n";
}
