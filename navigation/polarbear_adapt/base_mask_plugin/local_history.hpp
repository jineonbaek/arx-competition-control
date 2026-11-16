#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace arx_base_mask {
// World-stable 2D column history. Source stamps, never costmap update times,
// refresh real hits. Cleared entries remain as tombstones against buffered hits.
class LocalHistory {
 public:
  struct Key {
    int64_t x,y;
    bool operator==(const Key& other)const{return x==other.x && y==other.y;}
  };
  struct Hash {size_t operator()(const Key& k)const{
    return std::hash<int64_t>{}(k.x)^(std::hash<int64_t>{}(k.y)+0x9e3779b9+(std::hash<int64_t>{}(k.x)<<6));
  }};
  enum class State {Occupied,Unknown,Free};
  struct Entry {double hit,event;State state;};
  using Entries=std::unordered_map<Key,Entry,Hash>;
 private:
  double decay_,max_age_,last_now_=-1.;
  Entries entries_;
 public:
  explicit LocalHistory(double decay=5.,double max_age=.65):decay_(decay),max_age_(max_age){}
  bool fresh(double stamp,double now)const{
    return std::isfinite(stamp)&&std::isfinite(now)&&stamp>0.&&stamp<=now+1e-6&&now-stamp<=max_age_;
  }
  bool contains(Key k)const{return entries_.find(k)!=entries_.end();}
  const Entry& at(Key k)const{return entries_.at(k);}
  Entries& entries(){return entries_;}
  const Entries& entries()const{return entries_;}
  void erase(Key k){entries_.erase(k);}
  void clear(){entries_.clear();last_now_=-1.;}
  void adopt(Key k,double now){entries_.try_emplace(k,Entry{now,now,State::Occupied});}
  void hit(Key k,double stamp,double now){
    if(!fresh(stamp,now))return;
    auto it=entries_.find(k);
    if(it==entries_.end()){entries_.emplace(k,Entry{stamp,stamp,State::Occupied});return;}
    if(stamp>it->second.event+1e-9)it->second={stamp,stamp,State::Occupied};
  }
  void through(Key k,double stamp,double now){
    auto it=entries_.find(k);
    if(it==entries_.end()||!fresh(stamp,now)||stamp<=it->second.hit+1e-9||stamp<it->second.event)return;
    it->second.event=stamp;it->second.state=State::Free;
  }
  void advance(double now){
    if(last_now_>=0.&&now<last_now_-1e-6){
      for(auto& p:entries_)p.second={now,now-1e-6,State::Unknown};
    }
    last_now_=now;
    for(auto& p:entries_)if(p.second.state!=State::Free && now-p.second.hit>=decay_-1e-9)
      p.second.state=State::Free;
  }
};
}
