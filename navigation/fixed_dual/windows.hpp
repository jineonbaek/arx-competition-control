#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>
struct Sample {int64_t time; int source; float x=0,y=0,z=0; uint8_t reflectivity=0,tag=0,line=0;};
struct Window {int64_t start,end;std::vector<Sample> points;};
inline bool sufficient_sources(const Window& window,const std::array<bool,2>& enabled) {
  std::array<std::size_t,2> counts{};
  for(const auto& point:window.points) {
    if(point.source<0 || point.source>1)throw std::runtime_error("invalid source index");
    ++counts[point.source];
  }
  bool any=false;
  for(int source=0;source<2;++source)if(enabled[source]) {
    any=true;if(counts[source]<30)return false;
  }
  return any;
}
class Windows {
 public:
  std::array<bool,2> enabled{true,true};
  std::array<int64_t,2> first{},latest{};
  std::vector<Sample> pending;
  int64_t start=0,retained_after=0;
  size_t expired_points=0;
  void set_enabled(std::array<bool,2> value){
    if(value==enabled)return;
    enabled=value;first={};latest={};pending.clear();start=0;retained_after=0;
  }
  void add(int s,int64_t stamp,std::vector<Sample> points){
    if(!enabled[s])return;
    if(latest[s] && stamp<=latest[s])throw std::runtime_error("nonmonotonic lidar clock");
    if(!first[s])first[s]=stamp;
    latest[s]=stamp;pending.insert(pending.end(),points.begin(),points.end());
    // drain() may wait for an enabled source or a fresh IMU. Data older than
    // the frontend's 500 ms output horizon cannot become a usable window.
    // Bound that backlog without disguising clock faults or emitting one-sided
    // windows. sufficient_sources() still checks each recovered output.
    retained_after=std::max(retained_after,std::max(latest[0],latest[1])-500000000);
    auto end=std::remove_if(pending.begin(),pending.end(),[this](const Sample& p){return p.time<retained_after;});
    expired_points+=std::distance(end,pending.end());pending.erase(end,pending.end());
    if(pending.size()>500000)throw std::runtime_error("incomplete source window queue overflow");
  }
  std::optional<Window> pop(int64_t width=100000000){
    if(!enabled[0]&&!enabled[1])return {};
    int64_t earliest=0,watermark=INT64_MAX;
    for(int s=0;s<2;s++)if(enabled[s]){
      if(!first[s])return {};
      earliest=std::max(earliest,first[s]);watermark=std::min(watermark,latest[s]);
    }
    if(!start)start=((earliest+width-1)/width)*width;
    start=std::max(start,((retained_after+width-1)/width)*width);
    if(watermark<start+width)return {};
    Window out{start,start+width,{}};
    std::vector<Sample> next;
    for(auto &p:pending){if(p.time>=out.end)next.push_back(p);else if(p.time>=out.start)out.points.push_back(p);}
    pending.swap(next);start=out.end;
    std::stable_sort(out.points.begin(),out.points.end(),[](auto &a,auto &b){return a.time<b.time;});
    return out;
  }
};
