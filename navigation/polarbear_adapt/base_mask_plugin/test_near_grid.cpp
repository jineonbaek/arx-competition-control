#include "near_local_grid.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace arx_base_mask::near_local;
int main(){
 std::vector<unsigned char> a(100*100,0);Grid g{100,100,.1,-5.,-5.,0.,&a};
 assert(valid(g));assert(sample(g,.05,.05,.02)==0);
 a[50*100+50]=254;assert(sample(g,.05,.05,.02)==254);
 assert(sample(g,.105,.05,.02)==254); // overlap thin obstacle at adjacent pixel edge
 assert(sample(g,.115,.05,.02)==0);
 a[50*100+50]=255;assert(sample(g,.05,.05,.02)==255);
 assert(sample(g,4.999,0,.02)==255);assert(sample(g,6,0,.02)==255);
 Grid rotated=g;rotated.x=5.;rotated.yaw=std::acos(-1.)/2;assert(sample(rotated,-.05,.05,.02)==255);
 a[50*100+50]=253;assert(sample(rotated,-.05,.05,.02)==253);
 Grid shifted=g;shifted.x=-4.;shifted.y=-3.;assert(sample(shifted,1.05,2.05,.02)==253);
 assert(touches_disk(2.005,0,.02,0,0,2));assert(!touches_disk(2.02,0,.02,0,0,2));
 assert(!valid(Grid{100,101,.1,0,0,0,&a}));assert(!valid(Grid{100,100,0,0,0,0,&a}));
 assert(sample(g,NAN,0,.02)==255);
 // AABB-only oversampling would pull this non-overlapping corner cell in.
 std::vector<unsigned char> b(10000,0);Grid q{100,100,.1,0.,-std::sqrt(50.),std::acos(-1.)/4,&b};
 b[50*100+51]=254;assert(sample(q,0,.07,.02)==0);
 std::cout<<"PASS near-local cell coverage, rotation, rolling origin, boundary, raw costs and unavailable handling\n";
}
