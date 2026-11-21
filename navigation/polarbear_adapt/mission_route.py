"""Typed route policy. No ROS, transport, hardware or task-action semantics."""
import copy,math,uuid
from chassis_pose_policy import MOTION_MAX_AGE_NS
from t2_scope import t2_pair_indices

def uid():return str(uuid.uuid4())
def point(value):
 if not isinstance(value,dict) or set(value)-{'id','type','pose','point_id','pass_radius_m','name','test_task_id'}:raise ValueError('invalid point fields')
 kind=value.get('type','stop');pose=value.get('pose')
 if kind not in ('stop','via','task','special') or not isinstance(pose,list) or len(pose)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in pose):raise ValueError('invalid point pose/type')
 ident=value.get('id',uid())
 if not isinstance(ident,str) or not 1<=len(ident)<=80:raise ValueError('invalid point id')
 p=dict(id=ident,type=kind,pose=list(map(float,pose)))
 if 'name' in value:
  name=value['name']
  if not isinstance(name,str) or len(name)>64 or any(ord(c)<32 or ord(c)==127 for c in name):raise ValueError('invalid point name')
  if name.strip():p['name']=name.strip()
 if 'test_task_id' in value:
  if kind!='special' or value['test_task_id'] not in ('point_1','point_2'):raise ValueError('invalid test task marker')
  p['test_task_id']=value['test_task_id']
 if kind=='task':
  if value.get('point_id') not in ('point_1','point_2'):raise ValueError('invalid task point_id')
  p['point_id']=value['point_id']
 if kind=='via':
  radius=value.get('pass_radius_m',.10)
  if type(radius) not in (int,float) or not math.isfinite(radius) or not .02<=radius<=.5:raise ValueError('invalid via radius')
  p['pass_radius_m']=float(radius)
 return p

def distance(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])
def segment_hit(a,b,p,r):
 dx,dy=b[0]-a[0],b[1]-a[1];square=dx*dx+dy*dy
 u=0. if square<1e-18 else max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/square))
 return u if distance((a[0]+u*dx,a[1]+u*dy),p)<=r else None

def path_covers_vias(path,vias):
 if not vias:return True
 if not len(path):return False
 cursor=0;fraction=0.
 for v in vias:
  found=False
  for i in range(cursor,max(1,len(path)-1)):
   a,b=path[i],path[min(i+1,len(path)-1)]
   u=segment_hit(a,b,v['pose'],v['pass_radius_m'])
   if u is not None and (i>cursor or u+1e-9>=fraction):cursor,fraction=i,u;found=True;break
  if not found:return False
 return True

class MissionRoute:
 def __init__(self,map_id):self.map_id=map_id;self.reset('draft')
 def reset(self,reason='draft'):
  self.points=[];self.index=0;self.running=False;self.state=reason;self.route_id=uid();self.mission_id=None;self.target_index=None;self.sample=None
  self.mode='navigation';self.direct_source=None;self.execution_end=None;self.t2_index=None
 def add(self,value):
  if self.running:raise ValueError('route running')
  if self.state!='draft':self.reset()
  p=point(value)
  if len(self.points)>=200 or any(x['id']==p['id'] for x in self.points):raise ValueError('duplicate or too many points')
  self.points.append(p);return True
 def definition(self):return dict(schema_version=2,route_id=self.route_id,map_id=self.map_id,points=copy.deepcopy(self.points))
 def load(self,value,allow_map_change=False):
  if self.running:raise ValueError('route running')
  if not isinstance(value,dict) or set(value)!={'schema_version','route_id','map_id','points'} or type(value['schema_version']) is not int or value['schema_version'] not in (1,2) or not isinstance(value['points'],list) or len(value['points'])>200:raise ValueError('invalid route definition')
  if value['map_id']!=self.map_id and not allow_map_change:raise ValueError('map identity mismatch')
  parsed=[point(p) for p in value['points']]
  if len({p['id'] for p in parsed})!=len(parsed):raise ValueError('duplicate point id')
  route_id=str(uuid.UUID(value['route_id']))
  self.reset();self.points=parsed;self.route_id=route_id
 def can_enter_direct(self,pose,after_task=False):
  i=self.index+(1 if after_task else 0)
  if pose is None or len(pose)!=3 or not all(math.isfinite(v) for v in pose):return False
  if after_task and i<self._execution_limit() and self.points[i]['type'] in ('task','special'):return True
  return (i+1<self._execution_limit() and self.points[i]['type']=='special' and self.points[i+1]['type']=='special'
          and distance(pose,self.points[i]['pose'])<=.10)
 def _direct_end(self,start):
  end=start
  while (end+1<self._execution_limit() and self.points[end+1]['type']=='special'
         and not self.points[end].get('test_task_id') and not self.points[end+1].get('test_task_id')):end+=1
  return end
 def _group(self,pose=None,after_task=False,direct_start=False):
  self.mode='navigation';self.direct_source=None
  if self.index>=self._execution_limit():self.running=False;self.state='completed';self.target_index=None;return None
  self.state='running';self.sample=None
  if self.points[self.index]['type']=='task':
   self.mode='task_direct';self.target_index=self.index
   return copy.deepcopy(self.points[self.index])
  if (self.points[self.index].get('test_task_id') or (after_task and self.points[self.index]['type']=='special')
      or (direct_start and not self.can_enter_direct(pose))):
   # Competition entry always uses straight control, including near/non-special first points.
   self.direct_source=dict(pose=list(pose));self.mode='direct';self.target_index=self.index
   if self.points[self.index]['type']=='special':self.target_index=self._direct_end(self.target_index)
   return copy.deepcopy(self.points[self.target_index])
  if self.can_enter_direct(pose):
   self.direct_source=copy.deepcopy(self.points[self.index])
   self.index+=1;self.mode='direct';self.target_index=self.index
   self.target_index=self._direct_end(self.target_index)
   return copy.deepcopy(self.points[self.target_index])
  self.target_index=next((i for i in range(self.index,self._execution_limit()) if self.points[i]['type']!='via'),self._execution_limit()-1)
  if (self.points[self.target_index]['type']=='task' or self.points[self.target_index].get('test_task_id')) and self.target_index>self.index:self.target_index-=1
  return copy.deepcopy(self.points[self.target_index])
 def _execution_limit(self):return self.execution_end if self.execution_end is not None else len(self.points)
 def required_task_ids(self):
  if not self.running:return set()
  if self.t2_index is not None:return {'point_2'} if self.index<=self.t2_index else set()
  return {p['point_id'] for p in self.points[self.index:self._execution_limit()] if p['type']=='task'}
 def start_t2(self,available=(),pose=None):
  if self.running:raise ValueError('route running')
  if self.state not in ('draft','cancelled','completed'):raise ValueError('请先结束原导航任务')
  entry,end=t2_pair_indices(self.points)
  if 'point_2' not in available:raise ValueError('task worker unavailable: point_2')
  if pose is None or len(pose)!=3 or not all(math.isfinite(v) for v in pose):raise ValueError('valid T2 entry pose required')
  self.execution_end=len(self.points);self.t2_index=end;self.index=entry;self.running=True;self.mission_id=uid()
  self.state='running';self.sample=None;self.mode='direct';self.direct_source=dict(pose=list(pose));self.target_index=entry
  return copy.deepcopy(self.points[entry])
 def start(self,available=(),pose=None,direct_start=False):
  if self.running or self.state!='draft' or not self.points:return None
  if any(p.get('test_task_id') for p in self.points) and any(p['type']=='task' for p in self.points):raise ValueError('mixed test and real tasks are not executable')
  needed={p['point_id'] for p in self.points if p['type']=='task'}
  if needed-set(available):raise ValueError('task worker unavailable: '+','.join(sorted(needed-set(available))))
  if direct_start and (pose is None or len(pose)!=3 or not all(math.isfinite(v) for v in pose)):raise ValueError('valid competition entry pose required')
  self.execution_end=None;self.t2_index=None;self.running=True;self.mission_id=uid();return self._group(pose,direct_start=direct_start)
 def direct_points(self):
  return copy.deepcopy(self.points[self.index:self.target_index+1]) if self.running and self.mode in ('direct','task_direct') and self.target_index is not None else []
 def advance_direct(self,point_id):
  if not self.running or self.mode!='direct' or self.state!='running' or self.index>=self.target_index or self.points[self.index]['id']!=point_id:raise ValueError('invalid special advance')
  self.index+=1
 def vias(self):
  if not self.running or self.target_index is None or self.state!='running' or self.mode in ('direct','task_direct'):return []
  end=self.target_index
  if self.points[end]['type']=='via' and end+1<self._execution_limit() and (self.points[end+1]['type']=='task' or self.points[end+1].get('test_task_id')):end+=1
  return copy.deepcopy(self.points[self.index:end])
 def observe(self,pose,odom,stamp):
  if not self.vias():return False
  if not all(math.isfinite(v) for v in (*pose,*odom,stamp)):raise ValueError('invalid pose')
  previous=self.sample
  if previous is not None:
   dt=stamp-previous[0]
   if dt<=0:return False
   map_step=distance(pose,previous[1]);odom_step=distance(odom,previous[2])
   yaw=lambda a,b:abs(math.atan2(math.sin(a-b),math.cos(a-b)))
   map_yaw=yaw(pose[2],previous[1][2]);odom_yaw=yaw(odom[2],previous[2][2])
   body=lambda a,b:(math.cos(a[2])*(b[0]-a[0])+math.sin(a[2])*(b[1]-a[1]),-math.sin(a[2])*(b[0]-a[0])+math.cos(a[2])*(b[1]-a[1]))
   mismatch=distance(body(previous[1],pose),body(previous[2],odom))
   if max(map_step,odom_step)>max(.06,.45*dt) or mismatch>.04 or max(map_yaw,odom_yaw)>max(.08,.6*dt) or abs(map_yaw-odom_yaw)>.04:
    self.sample=None;self.running=False;self.state='localization_jump';raise ValueError('localization jump')
   if dt>MOTION_MAX_AGE_NS/1e9:
    self.sample=(stamp,list(pose),list(odom));return False
  self.sample=(stamp,list(pose),list(odom))
  a=previous[1] if previous is not None else pose
  p=self.points[self.index]
  if segment_hit(a,pose,p['pose'],p['pass_radius_m']) is not None:self.index+=1;return True
  return False
 def finish(self,outcome,pose=None):
  if not self.running:return None
  if outcome!='arrived_stopped':self.running=False;self.state=outcome;return None
  if self.state!='running':return None
  if self.vias():raise ValueError('unvisited via before terminal arrival')
  if self.mode=='direct' and self.index<self.target_index:raise ValueError('unvisited special before terminal arrival')
  p=self.points[self.target_index];self.index=self.target_index
  if self.t2_index is not None and self.index==self.t2_index-1:
   # The predecessor is a navigation location only, regardless of saved type.
   self.index+=1;return self._group(pose)
  if p['type']=='task':self.state='waiting_task';return None
  if p.get('test_task_id'):
   self.index+=1;return self._group(pose,after_task=True)
  if self.mode=='navigation' and p['type']=='special' and self.can_enter_direct(pose):return self._group(pose)
  self.index+=1;return self._group(pose)
 def release_task(self,pose=None):
  if not self.running or self.state!='waiting_task':return None
  if self.index+1<self._execution_limit() and self.points[self.index+1]['type']=='special':
   if pose is None or len(pose)!=3 or not all(math.isfinite(v) for v in pose):raise ValueError('actual post-task pose required')
  self.index+=1;return self._group(pose,after_task=True)
 def cancel(self):self.running=False;self.state='cancelled';self.sample=None
