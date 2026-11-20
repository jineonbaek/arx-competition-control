#pragma once
#include <nlohmann/json.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
// Reporting only: never read feedback as a substitute for a commanded target.
inline void append_reference_feedback(nlohmann::json& status,
    const std::vector<double>& reference,double clamp,bool from_trajectory){
  if(reference.size()!=6 || !std::isfinite(clamp) ||
      !std::all_of(reference.begin(),reference.end(),[](double v){return std::isfinite(v);}))return;
  status["joint_reference_rad"]=reference;
  status["gripper_target_raw"]=clamp;
  status["reference_from_trajectory"]=from_trajectory;
}
