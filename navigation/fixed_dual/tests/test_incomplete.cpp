#include "../windows.hpp"
#include <cassert>
#include <iostream>
int main(){
  Window bad{100,200,{}};
  for(int i=0;i<50;i++)bad.points.push_back(Sample{100+i,0});
  assert(!sufficient_sources(bad,{true,true}));
  Window good=bad;
  for(int i=0;i<30;i++)good.points.push_back(Sample{100+i,1});
  assert(sufficient_sources(good,{true,true}));
  assert(sufficient_sources(bad,{true,false}));
  assert(!sufficient_sources(good,{false,false}));
  // Rejecting one window must not reset the fusion clock or drop the next full one.
  Windows clock;clock.first={100,100};clock.latest={400,400};clock.start=100;
  clock.pending=bad.points;
  for(auto point:good.points){point.time+=100;clock.pending.push_back(point);}
  auto first=clock.pop(100);assert(first && !sufficient_sources(*first,clock.enabled));
  auto second=clock.pop(100);assert(second && sufficient_sources(*second,clock.enabled));
  assert(second->start==200 && second->end==300);
  std::cout<<"PASS: partial window rejected, next complete dual window preserved\n";
}
