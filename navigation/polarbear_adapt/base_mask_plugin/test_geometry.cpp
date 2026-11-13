#include "geometry.hpp"
#include <iostream>
#include <limits>
void check(bool b,const char* why){if(!b)throw std::runtime_error(why);}
int main(){
  using namespace arx_base_mask;
  Geometry shape("POLYGON ((-.32 -.32,.32 -.32,.32 .32,-.32 .32,-.32 -.32))");
  Pose p{0,0,0,10};
  check(shape.covers(0,0,.1,p),"central square was not covered");
  check(!shape.covers(.35,0,.1,p),"boundary crossing square was cleared");
  check(shape.covers(1,2,.1,{1,2,1.5707963267948966,10}),"rotated translated square missing");
  check(!shape.covers(0,0,.1,{1,2,0,10}),"old square covered after move");
  check(p.fresh(10.45)&&!p.fresh(10.66)&&!p.fresh(9.89),"measured TF lag or timeout validity failed");
  Window w;w.update(p,10.);w.update(Pose{1,0,0,10.1},10.1);
  double a=100,b=100,c=-100,d=-100;w.bounds(.5,&a,&b,&c,&d);
  check(a==-.5&&c==1.5,"old and new regions not both refreshed");
  w.update(std::nullopt,10.2);a=b=100;c=d=-100;w.bounds(.5,&a,&b,&c,&d);
  check(!w.current&&a==.5&&c==1.5,"TF loss failed to restore old region");
  w.update(Pose{0,0,0,9},10.2);check(!w.current,"stale pose accepted");
  std::cout<<"PASS: complete-cell geometry, move, rotation, stale TF and restoration bounds\n";
}
