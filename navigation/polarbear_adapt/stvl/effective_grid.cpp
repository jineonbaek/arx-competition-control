// Source-stamped finite-ray votes for STVL's actual OpenVDB cells.
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
#include <algorithm>
#include <fstream>
#include <stdexcept>

extern "C" {
void *new_history_size(double,double,double);
void free_history(void *);
void update_bounds(void *,int,const int32_t *,const double *);
void erase_keys(void *,int,const int32_t *);
int query_rays(void *,const double *,int,const double *,int32_t *,int);
}

namespace volume_grid {
using Point = std::array<double, 3>;
using Key = openvdb::Coord;
using Cells = std::unordered_set<Key>;

void SpatioTemporalVoxelGrid::StartEvidence() {
  _ray_index.reset(new_history_size(_voxel_size,_voxel_size,_voxel_size));
}

void SpatioTemporalVoxelGrid::Forget(const Key &k) const {
  if (!_ray_index) return;
  int32_t xyz[]{k.x(),k.y(),k.z()};
  erase_keys(_ray_index.get(),1,xyz);
  _observed.erase(k);
}

void SpatioTemporalVoxelGrid::RecordHit(const Key &k, double low, double high, int64_t ns) {
  if (_retired.count(k) && _retired.at(k) > ns) return;
  auto &cell = _observed[k];
  if (ns < cell.last_hit) return;
  if (!cell.last_hit) {cell.low=low; cell.high=high;}
  else {cell.low=std::min(cell.low,low); cell.high=std::max(cell.high,high);}
  cell.last_hit=ns;
  int32_t xyz[]{k.x(),k.y(),k.z()};
  double heights[]{cell.low,cell.high};
  update_bounds(_ray_index.get(),1,xyz,heights);
  MarkGridPoint(k, ns*1e-9);
}

void SpatioTemporalVoxelGrid::LoadEvidence(const std::string &file) {
  if (file.empty()) return;
  std::ifstream input(file);
  if (!input) throw std::runtime_error("STVL cannot read evidence seed: "+file);
  const int64_t now=_clock->now().nanoseconds();
  int x,y,z; double low,high; int64_t ns;
  while (input >> x >> y >> z >> low >> high >> ns) {
    if (ns<=now && now-ns < int64_t(_voxel_decay*1e9)) RecordHit(Key(x,y,z),low,high,ns);
  }
  if (!input.eof()) throw std::runtime_error("Invalid STVL history file");
}

void SpatioTemporalVoxelGrid::ApplyEvidence(const sensor_msgs::msg::PointCloud2 &cloud,
    size_t sensor, std::unordered_set<occupany_cell> &cleared) {
  const int64_t ns=rclcpp::Time(cloud.header.stamp).nanoseconds(), now=_clock->now().nanoseconds();
  if (sensor>=2 || ns<=_last_packet[sensor] || ns>now+25000000 || now-ns>650000000 ||
      cloud.height!=1 || !cloud.width || cloud.point_step!=16 ||
      cloud.row_step!=16ull*cloud.width || cloud.data.size()!=cloud.row_step || cloud.is_bigendian ||
      cloud.fields.size()!=4) return;
  const char *names[]{"x","y","z","kind"};
  for(size_t i=0;i<4;++i) {
    const auto &f=cloud.fields[i];
    if(f.name!=names[i] || f.offset!=4*i || f.count!=1 ||
       f.datatype!=sensor_msgs::msg::PointField::FLOAT32) return;
  }
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z"),type(cloud,"kind");
  if (*type!=0 || !std::isfinite(*x)||!std::isfinite(*y)||!std::isfinite(*z)) return;
  _last_packet[sensor]=ns;
  const Point origin{*x,*y,*z};
  std::vector<double> rays;
  Cells endpoints, crossed, owned_hits, vacated;
  std::unordered_map<Key,std::pair<double,double>> hits;
  for (++x,++y,++z,++type;x!=x.end();++x,++y,++z,++type) {
    if(!std::isfinite(*x)||!std::isfinite(*y)||!std::isfinite(*z)) continue;
    Key k(std::floor(*x/_voxel_size),std::floor(*y/_voxel_size),std::floor(*z/_voxel_size));
    if (*type==1) {
      rays.insert(rays.end(),{*x,*y,*z});
      if (_observed.count(k)) endpoints.insert(k);
    } else if (*type==2) {
      const double dist2=std::pow(*x-origin[0],2)+std::pow(*y-origin[1],2)+std::pow(*z-origin[2],2);
      if(dist2>64. || *z<.03 || *z>1.35) continue;
      auto bounds=hits.emplace(k,std::make_pair(*z,*z)).first;
      bounds->second.first=std::min(bounds->second.first,double(*z));
      bounds->second.second=std::max(bounds->second.second,double(*z));
      if (_observed.count(k)) endpoints.insert(k);
    } else if (*type==5) {
      owned_hits.insert(k);
    } else if (*type==6) {
      vacated.insert(k);
    }
  }
  if (!rays.empty() && !_observed.empty()) {
    if (_crossed_keys.empty()) _crossed_keys.resize(3*1024);
    int count;
    while ((count=query_rays(_ray_index.get(),origin.data(),rays.size()/3,rays.data(),
                             _crossed_keys.data(),_crossed_keys.size()/3))<0) {
      _crossed_keys.resize(2*_crossed_keys.size());
    }
    for(int i=0;i<count;++i) {
      Key k(_crossed_keys[3*i],_crossed_keys[3*i+1],_crossed_keys[3*i+2]);
      if (!endpoints.count(k)) crossed.insert(k);
    }
  }
  auto observe=[&](const Key &k,bool through) {
    auto it=_observed.find(k);
    if(it==_observed.end() || ns<it->second.last_hit) return;
    it->second.last_visible=std::max(ns,it->second.last_visible);
    auto &votes=it->second.frames[sensor];
    while (!votes.empty() && ns-votes.front().first>=1000000000LL) votes.pop_front();
    votes.emplace_back(ns,through);
    // Unknown frames never visit this voxel. Keep the streak for its lifetime.
    auto &streak=it->second.consecutive[sensor];
    streak=through ? std::min(4u,streak+1) : 0;
    const bool four=streak==4;
    const auto passes=std::count_if(votes.begin(),votes.end(),[](const auto &v){return v.second;});
    if (!four && !(votes.size()>=5 && 5*passes>=4*static_cast<int>(votes.size()))) return;
    const auto center=IndexToWorld(k);
    _retired[k]=ns;
    ClearGridPoint(k);
    cleared.emplace(center[0],center[1]);
  };
  for(const auto &k:endpoints) observe(k,false);
  for(const auto &k:crossed) observe(k,true);
  // A fresh classification marks only its actual coordinates, including after a clear.
  for (const auto &key : vacated) {
    const auto it = _observed.find(key);
    if (it == _observed.end() || it->second.trail_source != static_cast<int>(sensor) ||
        it->second.last_hit >= ns || endpoints.count(key) || hits.count(key)) continue;
    const auto centre = IndexToWorld(key);
    _retired[key] = ns;
    ClearGridPoint(key);
    cleared.emplace(centre[0], centre[1]);
  }
  for (const auto &item : hits) {
    const auto &key = item.first;
    const auto prior = _observed.find(key);
    if (prior != _observed.end() && prior->second.last_hit > ns) continue;
    const int source = owned_hits.count(key) ? static_cast<int>(sensor) : -1;
    const bool unchanged = prior == _observed.end() || prior->second.trail_source == source;
    RecordHit(key, item.second.first, item.second.second, ns);
    auto it = _observed.find(key);
    if (it != _observed.end() && it->second.last_hit == ns)
      it->second.trail_source = unchanged ? source : -1;
  }
  for(auto it=_retired.begin();it!=_retired.end();) {
    if(now-it->second>1000000000LL) it=_retired.erase(it); else ++it;
  }
}
} // namespace volume_grid
