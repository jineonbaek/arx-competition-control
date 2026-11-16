#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

struct ChassisOutput {
  std::string instance;
  std::uint64_t seq;
  double issued, ttl;
  std::array<double,3> sdk;
};

// Driver-side order is separate from all arm/lift epochs. Rejecting an input
// must not synthesize a global stop or renew the accepted output's watchdog.
class ChassisOutputGate {
 public:
  explicit ChassisOutputGate(std::string instance): instance_(std::move(instance)) {}
  bool accept(const ChassisOutput &m,double wall,bool ready) {
    if(m.instance!=instance_ || m.seq==0 || m.seq<=seq_ ||
       !std::isfinite(wall) || !std::isfinite(m.issued) || !std::isfinite(m.ttl) ||
       m.ttl<=0. || m.ttl>.3 || m.issued>wall+.1 || wall>=m.issued+m.ttl) return false;
    for(auto v:m.sdk)if(!std::isfinite(v))return false;
    const bool zero=m.sdk==std::array<double,3>{0.,0.,0.};
    if((!zero && !ready) || std::hypot(m.sdk[0],m.sdk[1])>2.+1e-9 ||
       std::abs(m.sdk[2])>2.+1e-9)return false;
    seq_=m.seq;return true;
  }
  std::uint64_t sequence()const{return seq_;}
 private:
  std::string instance_;
  std::uint64_t seq_=0;
};
