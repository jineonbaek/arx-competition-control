#ifndef ARX_LIDAR_VERTICAL_FOV_HPP
#define ARX_LIDAR_VERTICAL_FOV_HPP
#include <cmath>
namespace lidar_fov {
inline bool valid_bounds(double lower,double upper) {
 const double half_pi=std::acos(-1.)*.5;
 return std::isfinite(lower)&&std::isfinite(upper)&&lower>=-half_pi&&upper<=half_pi&&lower<upper;
}
inline bool inside(double z,double radial_squared,double padding,double lower,double upper) {
 if (!valid_bounds(lower,upper)||!std::isfinite(z)||!std::isfinite(radial_squared)||radial_squared<=0||!std::isfinite(padding)||padding<0)return false;
 const double r=std::sqrt(radial_squared),epsilon=1e-12;
 return std::atan2(z-padding,r)>=lower-epsilon&&std::atan2(z+padding,r)<=upper+epsilon;
}
}
#endif
