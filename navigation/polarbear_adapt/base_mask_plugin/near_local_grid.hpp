#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
namespace arx_base_mask::near_local {
struct Grid {
 unsigned width,height;double resolution,x,y,yaw;
 const std::vector<uint8_t>* data;
};
inline bool valid(const Grid& g){
 return g.width && g.height && uint64_t(g.width)*g.height<=10000000 &&
  g.data && g.data->size()==uint64_t(g.width)*g.height &&
  std::isfinite(g.resolution)&&g.resolution>0 && std::isfinite(g.x)&&std::isfinite(g.y)&&std::isfinite(g.yaw);
}
inline bool touches_disk(double x,double y,double resolution,double cx,double cy,double radius){
 const double dx=std::max(0.,std::abs(x-cx)-resolution*.5),dy=std::max(0.,std::abs(y-cy)-resolution*.5);
 return dx*dx+dy*dy<=radius*radius;
}
// All Local pixels with positive overlap of the complete target cell participate.
// Unknown or incomplete Local coverage remains unknown. Raw 0..255 is lossless.
inline uint8_t sample(const Grid& g,double wx,double wy,double resolution){
 if(!valid(g)||!std::isfinite(wx)||!std::isfinite(wy)||!std::isfinite(resolution)||resolution<=0)return 255;
 const double c=std::cos(g.yaw),s=std::sin(g.yaw);
 const double dx=wx-g.x,dy=wy-g.y;
 const double x=(c*dx+s*dy)/g.resolution,y=(-s*dx+c*dy)/g.resolution;
 const double h=resolution*.5/g.resolution,e=1e-9;
 const double extent=h*(std::abs(c)+std::abs(s));
 if(x-extent < -e || y-extent < -e || x+extent>g.width+e || y+extent>g.height+e)return 255;
 const int ax=std::max(0,int(std::floor(x-extent+e))),ay=std::max(0,int(std::floor(y-extent+e)));
 const int bx=std::min(int(g.width)-1,int(std::ceil(x+extent-e))-1),by=std::min(int(g.height)-1,int(std::ceil(y+extent-e))-1);
 uint8_t cost=0;bool covered=false;
 for(int j=ay;j<=by;++j)for(int i=ax;i<=bx;++i){
  const double px=x-(i+.5),py=y-(j+.5),r=h+.5*(std::abs(c)+std::abs(s));
  // Separating axes of the rotated target square and the Local pixel.
  if(std::abs(px*c-py*s)>=r-e || std::abs(px*s+py*c)>=r-e)continue;
  covered=true;cost=std::max(cost,(*g.data)[size_t(j)*g.width+i]);
 }
 return covered?cost:255;
}
}
