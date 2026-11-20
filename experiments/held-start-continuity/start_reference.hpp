#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <limits>

struct StartReference { std::vector<double> value; bool preserved; };

inline double choose_start_gripper_target(double requested,double lower,double upper) {
  if(!std::isfinite(requested) || !std::isfinite(lower) || !std::isfinite(upper) || lower>=upper)
    throw std::invalid_argument("Invalid gripper startup target");
  if(requested==1.)return std::numeric_limits<double>::quiet_NaN();
  if(requested<lower || requested>upper)throw std::invalid_argument("Gripper startup target outside limits");
  return requested;
}

inline StartReference choose_start_reference(const std::vector<double>& held,
                                              const std::vector<double>& actual,
                                              const std::vector<double>& first) {
  if (held.size()!=6 || actual.size()!=7 || first.size()!=6 ||
      !std::all_of(held.begin(),held.end(),[](double x){return std::isfinite(x);}) ||
      !std::all_of(actual.begin(),actual.end(),[](double x){return std::isfinite(x);}) ||
      !std::all_of(first.begin(),first.end(),[](double x){return std::isfinite(x);}))
    throw std::invalid_argument("Invalid trajectory starting reference");
  bool match=true;
  constexpr double rad_per_degree=0.017453292519943295;
  for (std::size_t i=0;i<6;++i)
    match=match && std::abs(first[i]-held[i])<=.05*rad_per_degree &&
                   std::abs(held[i]-actual[i])<=2.*rad_per_degree;
  if(match)return {held,true};
  return {std::vector<double>(actual.begin(),actual.begin()+6),false};
}
