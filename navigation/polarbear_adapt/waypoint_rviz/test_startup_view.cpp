#include "startup_view_policy.hpp"
#include <limits>
#include <stdexcept>
int main() {
  using arx_waypoints::startup_camera;
  auto check=[](bool ok){if(!ok)throw std::runtime_error("startup view policy failed");};
  auto a=startup_camera(-14.6,3.4,.2,1000);
  check(a&&a->x==-14.6&&a->y==3.4&&a->scale==100.);
  auto b=startup_camera(2.,-3.,.65,1600);
  check(b&&b->scale==160.); // Actual width/scale remains 10 m at any render size.
  check(!startup_camera(0.,0.,.651,1000));
  check(!startup_camera(0.,0.,-.101,1000));
  check(!startup_camera(std::numeric_limits<double>::quiet_NaN(),0.,0.,1000));
  check(!startup_camera(0.,0.,0.,0));
}
