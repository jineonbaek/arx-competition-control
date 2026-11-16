#ifndef ARX_VOXEL_PROJECTION_HPP
#define ARX_VOXEL_PROJECTION_HPP
#include <algorithm>
#include <cmath>
#include <limits>
namespace arx_projection {
struct Window {int x0=0,y0=0,x1=0,y1=0;};
inline double edge(double value) {
 const double nearest=std::round(value);
 const double tolerance=64*std::numeric_limits<double>::epsilon()*std::max(1.,std::abs(value));
 return std::abs(value-nearest)<=tolerance?nearest:value;
}
// Half-open cell range covering every positive-area XY overlap, clipped to map.
inline Window overlap(double x,double y,double voxel,double ox,double oy,double res,unsigned width,unsigned height) {
 if (!(std::isfinite(x)&&std::isfinite(y)&&std::isfinite(voxel)&&std::isfinite(ox)&&std::isfinite(oy)&&std::isfinite(res)) || voxel<=0 || res<=0 || !width || !height)return {};
 const double half=voxel*.5,lx=edge((x-half-ox)/res),ly=edge((y-half-oy)/res),hx=edge((x+half-ox)/res),hy=edge((y+half-oy)/res);
 if(hx<=0 || hy<=0 || lx>=width || ly>=height)return {};
 Window w;
 w.x0=lx<=0?0:static_cast<int>(std::floor(lx));w.y0=ly<=0?0:static_cast<int>(std::floor(ly));
 w.x1=hx>=width?width:static_cast<int>(std::ceil(hx));w.y1=hy>=height?height:static_cast<int>(std::ceil(hy));
 return w;
}
}
#endif
