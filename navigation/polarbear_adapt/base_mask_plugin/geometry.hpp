#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <geos_c.h>

namespace arx_base_mask {
struct Pose {
  double x,y,yaw,stamp;
  bool fresh(double now) const {
    return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(yaw)&&
           std::isfinite(stamp)&&std::isfinite(now)&&now-stamp>=-.1&&now-stamp<=.65;
  }
};
class Geometry {
  GEOSContextHandle_t context_=GEOS_init_r();
  GEOSGeometry* shape_=nullptr;
  const GEOSPreparedGeometry* prepared_=nullptr;
 public:
  double radius=0.;
  explicit Geometry(const std::string& wkt) {
    shape_=GEOSGeomFromWKT_r(context_,wkt.c_str());
    if(!shape_ || GEOSisValid_r(context_,shape_)!=1)throw std::runtime_error("Invalid chassis envelope");
    prepared_=GEOSPrepare_r(context_,shape_);
    double a,b,c,d;
    if(!prepared_ || !GEOSGeom_getExtent_r(context_,shape_,&a,&b,&c,&d))
      throw std::runtime_error("Invalid chassis extent");
    radius=std::hypot(std::max(std::abs(a),std::abs(c)),std::max(std::abs(b),std::abs(d)));
  }
  Geometry(const Geometry&)=delete;
  Geometry& operator=(const Geometry&)=delete;
  ~Geometry() {
    if(prepared_)GEOSPreparedGeom_destroy_r(context_,prepared_);
    if(shape_)GEOSGeom_destroy_r(context_,shape_);
    GEOS_finish_r(context_);
  }
  bool covers(double wx,double wy,double resolution,const Pose& pose) const {
    auto sequence=GEOSCoordSeq_create_r(context_,5,2);
    const int dx[]={-1,1,1,-1,-1},dy[]={-1,-1,1,1,-1};
    const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
    for(int k=0;k<5;++k) {
      const double x=wx+resolution*.5*dx[k]-pose.x;
      const double y=wy+resolution*.5*dy[k]-pose.y;
      GEOSCoordSeq_setXY_r(context_,sequence,k,c*x+s*y,-s*x+c*y);
    }
    auto ring=GEOSGeom_createLinearRing_r(context_,sequence);
    auto cell=GEOSGeom_createPolygon_r(context_,ring,nullptr,0);
    const bool covered=cell && GEOSPreparedCovers_r(context_,prepared_,cell)==1;
    if(cell)GEOSGeom_destroy_r(context_,cell);
    return covered;
  }
};
class Window {
 public:
  std::optional<Pose> current,previous;
  void update(std::optional<Pose> pose,double now) {
    previous=current;
    current=pose && pose->fresh(now) ? pose : std::nullopt;
  }
  void bounds(double radius,double* lo_x,double* lo_y,double* hi_x,double* hi_y) const {
    for(const auto& pose:{previous,current})if(pose) {
      *lo_x=std::min(*lo_x,pose->x-radius);*lo_y=std::min(*lo_y,pose->y-radius);
      *hi_x=std::max(*hi_x,pose->x+radius);*hi_y=std::max(*hi_y,pose->y+radius);
    }
  }
};
}
