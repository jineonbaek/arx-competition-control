#pragma once
#include "local_history.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav2_costmap_2d/voxel_layer.hpp>

namespace arx_base_mask {
// Shared implementation inside the existing VoxelLayer, not a costmap plugin.
class LocalDecay {
  std::unique_ptr<LocalHistory> history_;
  double resolution_=0.,anchor_x_=0.,anchor_y_=0.;
  int64_t offset_x_=0,offset_y_=0;
  using Key=LocalHistory::Key;
  std::unordered_map<Key,double,LocalHistory::Hash> through_;
 public:
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node,const std::string& name){
    const auto key=name+".history_decay_seconds";
    if(!node->has_parameter(key))node->declare_parameter(key,0.);
    const double seconds=node->get_parameter(key).as_double();
    if(!std::isfinite(seconds)||seconds<0.)throw std::runtime_error("Invalid history_decay_seconds");
    if(seconds>0.)history_=std::make_unique<LocalHistory>(seconds,.65);
  }
  bool enabled()const{return bool(history_);}
  void reset(){if(history_)history_->clear();resolution_=0.;through_.clear();}
  void grid(nav2_costmap_2d::Costmap2D& grid){
    if(!enabled())return;
    if(resolution_!=grid.getResolution()){
      reset();resolution_=grid.getResolution();anchor_x_=grid.getOriginX();anchor_y_=grid.getOriginY();
    }
    offset_x_=std::llround((grid.getOriginX()-anchor_x_)/resolution_);
    offset_y_=std::llround((grid.getOriginY()-anchor_y_)/resolution_);
  }
  Key key(unsigned int x,unsigned int y)const{return {offset_x_+x,offset_y_+y};}
  void begin(double now){if(enabled()){history_->advance(now);through_.clear();}}
  void forget(unsigned int x,unsigned int y){if(enabled())history_->erase(key(x,y));}
  void hit(unsigned int x,unsigned int y,double stamp,double now){if(enabled())history_->hit(key(x,y),stamp,now);}
  bool fresh(double stamp,double now)const{return enabled()&&history_->fresh(stamp,now);}
  void adopt(nav2_costmap_2d::Costmap2D& grid,double now){
    if(!enabled())return;
    for(unsigned int y=0;y<grid.getSizeInCellsY();++y)
      for(unsigned int x=0;x<grid.getSizeInCellsX();++x)
        if(grid.getCost(x,y)==nav2_costmap_2d::LETHAL_OBSTACLE)history_->adopt(key(x,y),now);
  }
  void crossed(unsigned int x,unsigned int y,double stamp){
    const auto k=key(x,y);
    if(history_->contains(k))through_[k]=std::max(through_[k],stamp);
  }
  template<class Clear>
  void apply(nav2_costmap_2d::Costmap2D& grid,double now,Clear clear,
             double* a,double* b,double* c,double* d){
    if(!enabled())return;
    history_->advance(now);
    auto& entries=history_->entries();
    for(auto it=entries.begin();it!=entries.end();){
      const int64_t x=it->first.x-offset_x_,y=it->first.y-offset_y_;
      if(x<0||y<0||x>=grid.getSizeInCellsX()||y>=grid.getSizeInCellsY()){
        it=entries.erase(it);continue;
      }
      const auto through=through_.find(it->first);
      // Native 3D raytrace must also have resolved this whole column as free.
      if(through!=through_.end()&&grid.getCost(x,y)==nav2_costmap_2d::FREE_SPACE)
        history_->through(it->first,through->second,now);
      const auto state=it->second.state;
      if(state!=LocalHistory::State::Occupied){
        const auto cost=state==LocalHistory::State::Free ? nav2_costmap_2d::FREE_SPACE : nav2_costmap_2d::NO_INFORMATION;
        if(grid.getCost(x,y)!=cost){
          double wx,wy;grid.mapToWorld(x,y,wx,wy);
          *a=std::min(*a,wx-resolution_);*b=std::min(*b,wy-resolution_);
          *c=std::max(*c,wx+resolution_);*d=std::max(*d,wy+resolution_);
        }
        clear(unsigned(x),unsigned(y),cost);
      }
      ++it;
    }
  }
  // Unknown is only used on clock discontinuity. Native Max ignores 255, so
  // explicitly carry it into a free-default master rather than silently freeing.
  void mergeUnknown(nav2_costmap_2d::Costmap2D& grid,int a,int b,int c,int d)const{
    if(!enabled())return;
    for(const auto& p:history_->entries())if(p.second.state==LocalHistory::State::Unknown){
      const int64_t x=p.first.x-offset_x_,y=p.first.y-offset_y_;
      if(x>=a&&y>=b&&x<c&&y<d)grid.setCost(x,y,nav2_costmap_2d::NO_INFORMATION);
    }
  }
};
}
