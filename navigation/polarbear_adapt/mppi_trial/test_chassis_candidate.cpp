// Offline behavioral tests. No actuator or communications interfaces.
#include "chassis_motion.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using V = ChassisMotion::Velocity;
void require(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}
struct Fixture {
  ChassisMotion motion{2., 2.};
  double now = 10.;
  Fixture() { motion.step(now, 100, true); }
  void tick(V v, double age = 0., bool ready = true) {
    now += .0025;
    motion.request(v, 1, now);
    motion.step(now, age, ready);
  }
  void settle(V v) { for (int n=0; n<600; ++n) tick(v); }
};

void corrections_keep_longitudinal_tracking() {
  Fixture f;
  for (int n=0; n<1600; ++n) {
    f.tick({-.3, (n/24)%2 ? .005 : -.005, (n/16)%2 ? .001 : -.001});
    if (n>240) require(std::abs(f.motion.velocity()[0]+.3)<1e-9,
        "minor lateral/yaw reversal interrupted longitudinal tracking");
  }
}
void each_axis_reverses_through_zero_with_hold() {
  for (int axis=0; axis<3; ++axis) {
    Fixture f;
    V start{.3,.2,.04}; f.settle(start);
    V reverse=start; reverse[axis]=-start[axis];
    double zero=-1, negative=-1;
    for (int n=0; n<800; ++n) {
      f.tick(reverse); const auto v=f.motion.velocity();
      if (v[axis]==0 && zero<0) zero=f.now;
      if (v[axis]<0 && negative<0) negative=f.now;
      for (int other=0; other<3; ++other) if (other!=axis)
        require(std::abs(v[other]-start[other])<1e-9,
                "unrelated axis changed during reversal");
    }
    require(zero>0 && negative-zero>=.2-1e-8,
            "reversed axis did not hold zero for 0.2 seconds");
    require(f.motion.velocity()==reverse,"axis failed to converge after hold");
  }
}
void combined_slew_limits() {
  Fixture f;
  const V targets[]={{.4,.3,.08},{-.3,.4,-.08},{.3,-.4,.08},{0,.2,0}};
  for (auto target: targets) for(int n=0;n<800;++n) {
    auto before=f.motion.velocity(); f.tick(target); auto after=f.motion.velocity();
    require(std::hypot(after[0]-before[0],after[1]-before[1])<=1.6*.0025+1e-9,
            "combined XY slew exceeded existing bound");
    require(std::abs(after[2]-before[2])<=1.6*.0025+1e-9,
            "yaw slew exceeded existing bound");
    require(std::hypot(after[0],after[1])<=.5+1e-9,"XY exceeded commanded envelope");
  }
}
void changing_lateral_target_cannot_starve_reversal() {
  Fixture f; f.settle({.3,.6,0});
  double zero=-1, negative=-1;
  for(int n=0;n<600;++n) {
    const auto before=f.motion.velocity();
    f.tick({-.3, (n/20)%3 ? .7 : .5, 0});
    const auto after=f.motion.velocity();
    if(after[0]==0 && zero<0) zero=f.now;
    if(after[0]<0 && negative<0) negative=f.now;
    require(std::hypot(after[0]-before[0],after[1]-before[1])<=1.6*.0025+1e-9,
            "changing-target reversal exceeded combined slew bound");
    require(std::hypot(after[0],after[1])<=.8+1e-9,
            "changing-target reversal exceeded command envelope");
  }
  require(zero>0 && negative-zero>=.2-1e-8,
          "changing lateral target starved longitudinal zero/hold/reversal");
  require(f.motion.velocity()[0]<-.29,"changing lateral target prevented reversal convergence");
}
void global_stop_and_fault_protection() {
  Fixture f; f.settle({.3,.1,.04});
  f.motion.stop(f.now); const double stopped=f.now;
  for(int n=0;n<300;++n) {
    f.motion.stop(f.now); f.tick({.3,.1,.04});
    require(f.motion.velocity()==V{},"movement bypassed global brake hold");
  }
  require(f.now-stopped<.8,"invalid brake test duration");
  f.now=stopped+.801; f.motion.step(f.now,10,true);
  require(f.motion.mode()==2,"repeated stop extended brake indefinitely");
  f.settle({.3,.1,.04}); f.tick({.3,.1,.04},.25);
  require(f.motion.velocity()==V{},"watchdog did not zero all axes");
  f.now+=1.; f.motion.step(f.now,10,true);
  f.settle({.3,.1,.04}); f.tick({.3,.1,.04},0,false);
  require(f.motion.mode()==2 && f.motion.velocity()==V{},"not-ready protection failed");
  f.tick({std::numeric_limits<double>::quiet_NaN(),0,0});
  require(f.motion.mode()==2 && f.motion.velocity()==V{},"NaN command was accepted");
  f.tick({2.,2.,0});
  require(f.motion.mode()==2 && f.motion.velocity()==V{},"out-of-range vector was accepted");
}
int main() {
  try {
    corrections_keep_longitudinal_tracking();
    each_axis_reverses_through_zero_with_hold();
    combined_slew_limits();
    changing_lateral_target_cannot_starve_reversal();
    global_stop_and_fault_protection();
    std::cout << "PASS: five offline behavior groups\n";
  } catch(const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n'; return 1;
  }
}
