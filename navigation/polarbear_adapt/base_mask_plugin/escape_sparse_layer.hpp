#pragma once
#include <mutex>
#include <chrono>
#include <cstring>
#include <unordered_set>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav2_msgs/msg/costmap.hpp>
namespace arx_base_mask {
class EscapeSparse {
 using Cloud=sensor_msgs::msg::PointCloud2;using Steady=std::chrono::steady_clock;
 std::mutex mutex_;Cloud::ConstSharedPtr message_;double floor_=-1.,last_now_=-1.,received_=0.;bool enabled_=false;
 std::vector<std::pair<double,double>> current_,previous_;
 rclcpp::Subscription<Cloud>::SharedPtr subscription_;
 rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr reference_;
 double reference_at_=-1.,frozen_stamp_=-1.,frozen_received_=0.;
 static double wall(){return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();}
 public:
 void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr n,const std::string&name,const std::string&frame){
  auto p=name+".escape_sparse_enabled";if(!n->has_parameter(p))n->declare_parameter(p,false);enabled_=n->get_parameter(p).as_bool();if(!enabled_)return;
  if(frame!="odom")throw std::runtime_error("Sparse exceptions must be Local-only");
  reference_=n->create_publisher<nav2_msgs::msg::Costmap>("/pb/escape/local_reference",rclcpp::QoS(1));
  subscription_=n->create_subscription<Cloud>("/pb/escape/sparse_lease",rclcpp::QoS(1),[this](Cloud::ConstSharedPtr m){
   std::lock_guard<std::mutex> lock(mutex_);bool ok=m->header.frame_id=="odom"&&!m->is_bigendian&&m->height==1&&m->width<=6&&m->point_step==12&&m->row_step==m->width*12&&m->data.size()==m->width*12&&m->fields.size()==3;
   for(unsigned i=0;ok&&i<3;++i)ok=m->fields[i].name==(i==0?"x":i==1?"y":"z")&&m->fields[i].offset==4*i&&m->fields[i].datatype==7&&m->fields[i].count==1;
   if(!ok){message_.reset();return;}
   double stamp=rclcpp::Time(m->header.stamp).seconds();if(stamp<=floor_)return;floor_=stamp;message_=m;received_=wall();
  });
 }
 void update(double now){
  if(!enabled_)return;previous_=current_;current_.clear();std::lock_guard<std::mutex> lock(mutex_);
  if(last_now_>=0&&now<last_now_)message_.reset();last_now_=now;
  if(!message_)return;double stamp=rclcpp::Time(message_->header.stamp).seconds();
  if(now-stamp<-.1||now-stamp>.35||wall()-received_>.35){message_.reset();return;}
  frozen_stamp_=stamp;frozen_received_=received_;
  for(unsigned i=0;i<message_->width;++i){float p[3];std::memcpy(p,message_->data.data()+12*i,12);if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2])){current_.clear();return;}current_.emplace_back(p[0],p[1]);}
 }
 void bounds(double radius,double*a,double*b,double*c,double*d)const{
  if(!enabled_)return;for(const auto*points:{&previous_,&current_})for(auto [x,y]:*points){*a=std::min(*a,x-radius);*b=std::min(*b,y-radius);*c=std::max(*c,x+radius);*d=std::max(*d,y+radius);}
 }
 void apply(nav2_costmap_2d::Costmap2D&g,const rclcpp::Clock::SharedPtr&clock){
  if(!enabled_||(current_.empty()&&previous_.empty()))return;
  const double now=clock->now().seconds();
  if(now-reference_at_>=.10||now<reference_at_){
   nav2_msgs::msg::Costmap m;m.header.frame_id="odom";m.header.stamp=clock->now();m.metadata.size_x=g.getSizeInCellsX();m.metadata.size_y=g.getSizeInCellsY();m.metadata.resolution=g.getResolution();m.metadata.origin.position.x=g.getOriginX();m.metadata.origin.position.y=g.getOriginY();m.metadata.origin.orientation.w=1.;m.data.assign(g.getCharMap(),g.getCharMap()+g.getSizeInCellsX()*g.getSizeInCellsY());reference_->publish(m);reference_at_=now;
  }
  if(now-frozen_stamp_>.35||now-frozen_stamp_<-.1||wall()-frozen_received_>.35)current_.clear();
  // Source VoxelLayer/history is never modified. Only this cycle's master copy.
  std::unordered_set<unsigned> requested,visited;const int w=g.getSizeInCellsX(),h=g.getSizeInCellsY();const double res=g.getResolution();
  for(auto [x,y]:current_){unsigned i,j;if(g.worldToMap(x,y,i,j))requested.insert(g.getIndex(i,j));}
  std::vector<std::vector<unsigned>> approved;
  for(unsigned seed:requested){
   if(visited.count(seed)||g.getCharMap()[seed]!=254)continue;
   std::vector<unsigned> group{seed};visited.insert(seed);
   for(size_t at=0;at<group.size()&&group.size()<=3;++at){int x=group[at]%w,y=group[at]/w;for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){int i=x+dx,j=y+dy;if(i<0||j<0||i>=w||j>=h)continue;unsigned k=g.getIndex(i,j);if(g.getCharMap()[k]==254&&!visited.count(k)){visited.insert(k);group.push_back(k);}}}
   if(group.size()>3||group.size()*res*res>.0012+1e-9)continue;
   bool allow=true;std::unordered_set<unsigned> members(group.begin(),group.end());
   for(unsigned k:group)if(!requested.count(k))allow=false;
   const int reach=std::ceil((.10+std::sqrt(2.)*res)/res);
   for(unsigned k:group){int x=k%w,y=k/w;for(int j=std::max(0,y-reach);allow&&j<std::min(h,y+reach+1);++j)for(int i=std::max(0,x-reach);i<std::min(w,x+reach+1);++i){unsigned other=g.getIndex(i,j);if((g.getCharMap()[other]==254||g.getCharMap()[other]==255)&&!members.count(other)&&std::hypot(std::max(0,std::abs(i-x)-1)*res,std::max(0,std::abs(j-y)-1)*res)<.10-1e-8){allow=false;break;}}}
   if(allow)approved.push_back(group);
  }
  if(approved.size()<=2)for(const auto&group:approved)for(unsigned k:group)g.getCharMap()[k]=0;
 }
};
}
