"""Cooperative navigation/task handshake, exact identity and durable interruption."""
import copy,json,math,os,secrets,threading,time,uuid
from pathlib import Path
from route_readiness import route_readiness
from task_deadline import T2_TIMEOUT_S

class ProtocolError(ValueError):pass

def uuid_string(value):
 try:return str(uuid.UUID(value))
 except (ValueError,TypeError,AttributeError):raise ProtocolError('invalid UUID')

def strict_json(raw):
 def pairs(items):
  out={}
  for k,v in items:
   if k in out:raise ProtocolError('duplicate JSON key')
   out[k]=v
  return out
 def bad(_):raise ProtocolError('nonfinite JSON')
 try:return json.loads(raw,object_pairs_hook=pairs,parse_constant=bad)
 except (TypeError,UnicodeError,json.JSONDecodeError) as e:raise ProtocolError(str(e))

class HandoffHub:
 def __init__(self,journal,clock=time.monotonic):
  self.path=Path(journal);self.clock=clock;self.lock=threading.RLock();self.worker=None;self.active=None;self.requests={};self.began=0.
  self.events=[];self.navigation_check=None;self.route=None;self.task_started=None
  if self.path.exists():
   value=strict_json(self.path.read_bytes());self.active=value.get('active');self.requests=value.get('requests',{});self.events=value.get('events',[])[-128:]
   if self.active and self.active['status']!='acknowledged':
    self.active.update(status='needs_attention',error='navigation_restart');self._record('needs_attention');self._save()
  else:self._save()
 def _save(self):
  self.path.parent.mkdir(parents=True,exist_ok=True)
  tmp=self.path.with_suffix('.tmp')
  with os.fdopen(os.open(tmp,os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600),'w') as f:
   json.dump(dict(active=self.active,requests=self.requests,events=self.events),f,allow_nan=False);f.flush();os.fsync(f.fileno())
  tmp.replace(self.path)
 def _record(self,event,request_id=None):
  a=self.active or {}
  self.events.append(dict(event=event,request_id=request_id,at_unix_ms=time.time_ns()//1000000,**{k:a.get(k) for k in ('mission_id','visit_id','point_id','status','outcome','error')}))
  self.events=self.events[-128:]
 def set_navigation_check(self,stage,reason):
  # Observation only: HTTP never invokes or bypasses the ROS motion gate.
  with self.lock:self.navigation_check=dict(stage=stage,reason=reason,visit_id=(self.active or {}).get('visit_id'),at_unix_ms=time.time_ns()//1000000)
 def blocking(self):
  with self.lock:return self.active is not None and self.active['status']!='acknowledged'
 def _valid(self,token):
  if not self.worker or not secrets.compare_digest(str(token),self.worker['token']):raise ProtocolError('worker authentication failed')
  if self.clock()-self.worker['seen']>2.:raise ProtocolError('worker lease expired; register again')
 def register(self,value):
  with self.lock:
   self.tick()
   if not isinstance(value,dict) or set(value)!={'worker_id','instance_id','point_ids'}:raise ProtocolError('invalid registration')
   if value['worker_id']!='battery' or not isinstance(value['point_ids'],list) or not value['point_ids'] or not set(value['point_ids'])<= {'point_1','point_2'}:raise ProtocolError('invalid worker/points')
   instance=uuid_string(value['instance_id'])
   if self.worker and self.worker['instance_id']==instance:
    if self.blocking() and set(value['point_ids'])!=set(self.worker['point_ids']):raise ProtocolError('active visit worker capabilities changed')
    self.worker['seen']=self.clock()
    a=self.active
    if (a and a['status']=='needs_attention' and a.get('error')=='worker_or_task_timeout'
        and a.get('lease_suspended_status')=='notified' and self.clock()-self.began<=600):
     a.update(status='notified',error=None,revision=a['revision']+1)
     a.pop('lease_suspended_status',None);self._record('notification_reconnected');self._save()
    return dict(token=self.worker['token'],lease_seconds=2.)
   if self.blocking():raise ProtocolError('uncertain visit blocks worker replacement')
   if self.worker and self.clock()-self.worker['seen']<=2.:raise ProtocolError('another worker is registered')
   self.worker=dict(value,instance_id=instance,token=secrets.token_urlsafe(32),seen=self.clock())
   return dict(token=self.worker['token'],lease_seconds=2.)
 def heartbeat(self,token):
  with self.lock:self._valid(token);self.worker['seen']=self.clock();return dict(ok=True)
 def available_points(self):
  with self.lock:return set(self.worker['point_ids']) if self.worker and self.clock()-self.worker['seen']<=2. else set()
 def set_route(self,definition,*,state,running,index):
  # Volatile diagnostics only: never restore a route or persist its coordinates.
  with self.lock:self.route=route_readiness(definition,state=state,running=running,index=index)
 def state(self):
  with self.lock:return dict(protocol_version=1,worker_ready=bool(self.available_points()),point_ids=sorted(self.available_points()),task=copy.deepcopy(self.active),events=copy.deepcopy(self.events),navigation_check=copy.deepcopy(self.navigation_check),route=copy.deepcopy(self.route))
 def begin(self,value):
  with self.lock:
   if self.blocking():raise ProtocolError('previous visit not acknowledged')
   if value.get('point_id') not in self.available_points():raise ProtocolError('task worker unavailable')
   if set(value)!={'mission_id','point_id','route_point_id','map_id','goal_pose','arrival_pose','pose_stamp_ns'}:raise ProtocolError('invalid arrival context')
   uuid_string(value['mission_id'])
   for key in ('goal_pose','arrival_pose'):
    if not isinstance(value[key],list) or len(value[key])!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in value[key]):raise ProtocolError('invalid arrival pose')
   self.active=copy.deepcopy(value);self.active.update(visit_id=str(uuid.uuid4()),status='notified',revision=1,navigation_ready=False,control_released=False,error=None)
   self.began=self.clock();self.task_started=None;self._record('notified');self._save();return copy.deepcopy(self.active)
 def poll(self,token):
  with self.lock:
   self._valid(token)
   if self.blocking():
    if not self.active.get('first_read_at_ms'):
     self.active['first_read_at_ms']=time.time_ns()//1000000;self._record('notification_read');self._save()
    return copy.deepcopy(self.active)
   return None
 def event(self,token,value):
  with self.lock:
   self.tick()
   self._valid(token)
   if not isinstance(value,dict) or not {'request_id','mission_id','visit_id','point_id','event'}<=set(value) or set(value)-{'request_id','mission_id','visit_id','point_id','event','navigation_ready','control_released','error'}:raise ProtocolError('invalid event fields')
   request=uuid_string(value['request_id']);payload=json.dumps(value,sort_keys=True,allow_nan=False)
   if request in self.requests:
    old=self.requests[request]
    if old['payload']!=payload:raise ProtocolError('request_id payload conflict')
    return copy.deepcopy(old['result'])
   a=self.active
   if a is None or any(value[k]!=a[k] for k in ('mission_id','visit_id','point_id')):raise ProtocolError('stale or wrong visit')
   status=a['status'];event=value['event']
   if event=='accept' and status=='notified':a['status']='running';self.task_started=self.clock()
   elif event=='heartbeat' and status in ('notified','running'):pass
   elif event=='complete' and status=='running':
    if value.get('navigation_ready') is not True or value.get('control_released') is not True:raise ProtocolError('completion requires released control and navigation readiness')
    a.update(status='resume_pending',control_released=True,navigation_ready=True)
   elif event=='timeout' and status in ('running','needs_attention'):
    if (a['point_id']!='point_2' or self.task_started is None or self.clock()-self.task_started<T2_TIMEOUT_S
        or value.get('navigation_ready') is not True or value.get('control_released') is not True):
     raise ProtocolError('T2 timeout requires180 seconds, confirmed recovery and released control')
    a.update(status='resume_pending',outcome='timed_out',control_released=True,navigation_ready=True,error='t2_task_timeout_180s')
   elif event=='fail' and status in ('notified','running'):
    a.update(status='needs_attention',error=str(value.get('error','task_failed'))[:300])
   elif event=='cancelled' and status=='cancelling' and value.get('control_released') is True:a.update(status='cancelled',control_released=True)
   else:raise ProtocolError('event not allowed in '+status)
   self.worker['seen']=self.clock();a['revision']+=1
   result=copy.deepcopy(a);self.requests[request]=dict(payload=payload,result=result)
   # Bounded to this process's most recent requests; visit identity is still authoritative.
   if len(self.requests)>2048:self.requests.pop(next(iter(self.requests)))
   self._record(event,request);self._save();return result
 def cancel(self):
  with self.lock:
   if self.blocking() and self.active['status']!='cancelling':
    self.active.update(status='cancelling',revision=self.active['revision']+1,error='operator_cancelled');self._record('cancelling');self._save()
 def operator_cancel_verified(self,visit_id):
  with self.lock:
   if not self.active or self.active['visit_id']!=visit_id or self.active['status']!='cancelling':raise ProtocolError('not cancelling this visit')
   self.active.update(status='cancelled',control_released=True,revision=self.active['revision']+1);self._record('cancelled');self._save()
 def tick(self):
  with self.lock:
   if self.active and self.active['status'] in ('notified','running') and (not self.available_points() or self.clock()-self.began>600):
    if self.active['status']=='notified' and self.clock()-self.began<=600:
     self.active['lease_suspended_status']='notified'
    self.active.update(status='needs_attention',error='worker_or_task_timeout',revision=self.active['revision']+1);self._record('needs_attention');self._save()
 def acknowledge(self,visit_id):
  with self.lock:
   if not self.active or visit_id!=self.active['visit_id'] or self.active['status'] not in ('resume_pending','cancelled'):raise ProtocolError('visit cannot be acknowledged')
   outcome=self.active.get('outcome','completed') if self.active['status']=='resume_pending' else 'cancelled'
   self.active.update(status='acknowledged',outcome=outcome,revision=self.active['revision']+1);self._record('acknowledged');self._save()
