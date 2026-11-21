"""Retain issued joint paths; reverse only the prefix reached by measured joints."""
import copy,threading
import numpy as np

def reverse_issued_path(history,current,tolerance_rad=np.deg2rad(2.),goal=None):
 current=np.asarray(current,float)
 if current.shape!=(6,) or not np.isfinite(current).all():raise ValueError('恢复起姿无效')
 if not history:return current[None,:]
 rows=[]
 for entry in history:
  points=np.asarray(entry['points'],float)
  if points.ndim!=2 or points.shape[1]!=6 or not len(points) or not np.isfinite(points).all():raise ValueError('原轨迹无效')
  if rows and np.max(abs(rows[-1]-points[0]))>tolerance_rad:raise ValueError('原回退轨迹不连续')
  rows.extend(points)
 path=np.asarray(rows)
 def project(q):
  matches=[]
  for i in range(max(1,len(path)-1)):
   a=path[i];b=path[min(i+1,len(path)-1)];d=b-a;u=0. if d@d<1e-18 else float(np.clip((q-a)@d/(d@d),0.,1.));point=a+u*d
   metric=float(np.max(abs(q-point)))
   if metric<=tolerance_rad:matches.append((metric,i,u,point))
  if not matches:return None
  # Prefer a physically identical earlier point to avoid reversing an already
  # completed withdrawal and re-entering the cabinet. Deviations are CAD checked.
  return matches[0]
 reached=project(current)
 if reached is None:raise ValueError('右臂实测偏离原已下发路径，不能盲目回退')
 stop=project(np.asarray(goal,float)) if goal is not None else None
 stop_reached=stop is not None and stop[1]+stop[2]<=reached[1]+reached[2]
 end=stop[1] if stop_reached else 0
 out=[current,*path[end+1:reached[1]+1][::-1]]
 out.append(stop[3] if stop_reached else path[0])
 arr=np.asarray(out,float);return arr[np.r_[True,np.any(np.abs(np.diff(arr,axis=0))>1e-12,axis=1)]]

class RecordingMotion:
 def __init__(self,motion,read_joints):self.motion=motion;self.read_joints=read_joints;self.paths=[];self.lock=threading.RLock()
 def __getattr__(self,name):return getattr(self.motion,name)
 def arm(self,segments,*args,**kwargs):
  # Loaded command references can include gravity compensation. Retain measured
  # joint samples, never treat an unexecuted/compensated command as a return pose.
  entry=dict(points=[list(self.read_joints())],complete=False)
  with self.lock:self.paths.append(entry)
  def sample():
   actual=list(self.read_joints())
   with self.lock:
    if np.max(abs(np.asarray(entry['points'][-1])-actual))>1e-12:entry['points'].append(actual)
  def wrap(guard):
   def observed(*a,**kw):
    result=guard(*a,**kw);sample();return result
   return observed
  if args:args=(wrap(args[0]),*args[1:])
  elif 'guard' in kwargs:kwargs=dict(kwargs,guard=wrap(kwargs['guard']))
  else:raise ValueError('恢复记录需要实际运动guard回调')
  result=self.motion.arm(segments,*args,**kwargs);sample()
  with self.lock:entry['complete']=True
  return result
 def snapshot_paths(self):
  with self.lock:return copy.deepcopy(self.paths)
