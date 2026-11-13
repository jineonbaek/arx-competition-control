// Finite-ray traversal for the 5 x 5 x 2.5 cm historical grid.
// Only cells touched by a retained beam are looked up in the history.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
namespace {
using Vec=std::array<double,3>;
using Key=std::array<int32_t,3>;
struct Hash {
  std::size_t operator()(const Key &k) const {
    uint64_t h=1469598103934665603ULL;
    for (auto v:k) {h^=static_cast<uint32_t>(v); h*=1099511628211ULL;}
    return h;
  }
};

constexpr double bin=.7*3.141592653589793/180.;
struct Beam {Vec dir;double length;};
struct History {
  Vec pitch;
  std::unordered_map<Key,std::array<double,2>,Hash> bounds;
};
Key key_at(const Vec &p,const Vec &pitch) {
  Key k;for(int i=0;i<3;++i) k[i]=int32_t(std::floor(p[i]/pitch[i]));
  return k;
}
bool spans(const Vec &origin,const Beam &beam,Vec lo,Vec hi) {
  double entry=0,exit=beam.length;
  for(int i=0;i<3;++i){
    if(std::abs(beam.dir[i])<1e-10){
      if(origin[i]<lo[i] || origin[i]>hi[i]) return false;
    }else{
      double a=(lo[i]-origin[i])/beam.dir[i],b=(hi[i]-origin[i])/beam.dir[i];
      entry=std::max(entry,std::min(a,b));exit=std::min(exit,std::max(a,b));
    }
  }
  return exit-entry>=.008;
}
bool crosses(const Vec &o,const Beam &beam,const Key &k,const std::array<double,2> &z,double leave,const Vec &pitch) {
  if(beam.length<=leave+.03) return false;
  Vec lo,hi;
  double norm2=0;
  for(int i=0;i<3;++i){
    lo[i]=k[i]*pitch[i];hi[i]=lo[i]+pitch[i];
    double d=.5*(lo[i]+hi[i])-o[i];norm2+=d*d;
  }
  if(norm2<=.35*.35 || norm2>=64.) return false;
  for(int i=0;i<2;++i){lo[i]+=.003;hi[i]-=.003;}
  for(double h:z) {
    lo[2]=std::max(k[2]*pitch[2],h-.012);
    hi[2]=std::min((k[2]+1)*pitch[2],h+.012);
    if(!spans(o,beam,lo,hi))return false;
  }
  return true;
}
}
extern "C" {
void* new_history_size(double x,double y,double z){return new History{{x,y,z},{}};}
void* new_history(){return new_history_size(.05,.05,.025);}
void free_history(void* h){delete static_cast<History*>(h);}
void update_bounds(void* h,int n,const int32_t *keys,const double *bounds){
  auto &map=static_cast<History*>(h)->bounds;
  for(int j=0;j<n;++j) map[{keys[3*j],keys[3*j+1],keys[3*j+2]}]={bounds[2*j],bounds[2*j+1]};
}
void erase_keys(void* h,int n,const int32_t* keys){
  auto &map=static_cast<History*>(h)->bounds;
  for(int j=0;j<n;++j)map.erase({keys[3*j],keys[3*j+1],keys[3*j+2]});
}
int query_rays(void* h,const double *origin,int n,const double *points,int32_t *out,int capacity){
  const auto &map=static_cast<History*>(h)->bounds;
  const auto &pitch=static_cast<History*>(h)->pitch;
  Vec o={origin[0],origin[1],origin[2]};
  std::unordered_map<int,Beam> nearest;
  for(int j=0;j<n;++j){
    Vec d;for(int i=0;i<3;++i)d[i]=points[3*j+i]-o[i];
    double xy=std::hypot(d[0],d[1]),len=std::hypot(xy,d[2]);
    if(!(len>.35 && len<=8.))continue;
    int a=static_cast<int>(std::floor(std::atan2(d[1],d[0])/bin));
    int e=static_cast<int>(std::floor(std::atan2(d[2],xy)/bin));
    int id=(a+300)*512+e+150;
    auto it=nearest.find(id);
    if(it!=nearest.end() && it->second.length<=len)continue;
    for(auto &x:d)x/=len;
    nearest[id]={d,len};
  }
  std::unordered_set<Key,Hash> passed;
  for(const auto &item:nearest){
    const auto &beam=item.second;
    Key k=key_at(o,pitch),step;Vec next,delta;
    for(int i=0;i<3;++i){
      const double v=beam.dir[i];step[i]=v>0?1:-1;
      if(std::abs(v)<1e-10){next[i]=delta[i]=std::numeric_limits<double>::infinity();continue;}
      next[i]=((k[i]+(v>0?1:0))*pitch[i]-o[i])/v;delta[i]=pitch[i]/std::abs(v);
    }
    for(;;){
      double leave=std::min({next[0],next[1],next[2]});
      if(leave+.03>=beam.length)break;
      auto it=map.find(k);
      if(it!=map.end() && !passed.count(k) && crosses(o,beam,k,it->second,leave,pitch))passed.insert(k);
      for(int i=0;i<3;++i)if(next[i]<=leave+1e-12){k[i]+=step[i];next[i]+=delta[i];}
    }
  }
  if(int(passed.size())>capacity)return -1;
  int j=0;for(auto &k:passed)for(int a:k)out[j++]=a;
  return static_cast<int>(passed.size());
}
}
