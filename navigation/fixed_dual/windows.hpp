#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>
struct Sample {int64_t time; int source; float x=0,y=0,z=0; uint8_t reflectivity=0,tag=0,line=0;};
struct Window {int64_t start,end;std::vector<Sample> points;};
class Windows {
 public:
  std::array<bool,2> enabled{true,true};
  std::array<int64_t,2> first{},latest{};
  std::vector<Sample> pending;
  int64_t start=0;
  void set_enabled(std::array<bool,2> value){
    if(value==enabled)return;
    enabled=value;first={};latest={};pending.clear();start=0;
  }
  void add(int s,int64_t stamp,std::vector<Sample> points){
    if(!enabled[s])return;
    if(latest[s] && stamp<=latest[s])throw std::runtime_error("nonmonotonic lidar clock");
    if(!first[s])first[s]=stamp;
    latest[s]=stamp;pending.insert(pending.end(),points.begin(),points.end());
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
    if(watermark<start+width)return {};
    Window out{start,start+width,{}};
    std::vector<Sample> next;
    for(auto &p:pending){if(p.time>=out.end)next.push_back(p);else if(p.time>=out.start)out.points.push_back(p);}
    pending.swap(next);start=out.end;
    std::stable_sort(out.points.begin(),out.points.end(),[](auto &a,auto &b){return a.time<b.time;});
    return out;
  }
};
