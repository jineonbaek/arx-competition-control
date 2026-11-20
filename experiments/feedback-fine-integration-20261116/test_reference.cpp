#include "reference_feedback.hpp"
#include <cassert>
#include <limits>
int main(){
  nlohmann::json s={{"ready",true}};
  append_reference_feedback(s,{0,1,2,3,4,5},-.8,true);
  assert(s.at("gripper_target_raw")==-.8);
  assert(s.at("joint_reference_rad").size()==6);
  assert(s.at("reference_from_trajectory")==true);
  nlohmann::json bad;
  append_reference_feedback(bad,{0,1},-.8,false);
  assert(!bad.contains("joint_reference_rad"));
  append_reference_feedback(bad,{0,1,2,3,4,5},std::numeric_limits<double>::quiet_NaN(),false);
  assert(!bad.contains("gripper_target_raw"));
}
