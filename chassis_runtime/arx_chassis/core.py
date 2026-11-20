"""Authoritative arbitration; no ROS, HTTP, SDK or hardware initialization."""
import math
import time
import uuid
from .scheduler import ChassisScheduler, InvalidChassis
Rejected = InvalidChassis

class Core:
    def __init__(self, output, *, wall=time.time, clock=time.monotonic, body=None):
        self.wall,self.clock=wall,clock
        self.instance=uuid.uuid4().hex
        self.driver=None;self.bound_instance='';self.applied={}
        self.clients={};self.resume_sessions={};self.output_connected=True;self.maintenance_client=None;self.maintenance_token=None
        self.body=body or dict(max_linear_m_s=1.6,max_angular_rad_s=1.8,manual_sdk_axes={'x':.92,'y':1.6})
        self.scheduler=ChassisScheduler(output,wall=wall,clock=clock,lease=1.)
        self.scheduler.not_before=wall()

    @property
    def ready(self):
        return bool(self.output_connected and self.driver and self.driver[1] and 0<=self.clock()-self.driver[2]<.25
                    and self.bound_instance==self.instance)

    def register(self, profile, resume_key=""):
        if profile not in ('manual','navigation','gamepad'):raise Rejected('unknown source profile')
        if not isinstance(resume_key,str) or len(resume_key)>256:raise Rejected('invalid resume key')
        previous=self.resume_sessions.get((profile,resume_key)) if resume_key else None
        if previous in self.clients:return previous
        client=uuid.uuid4().hex
        if resume_key:self.resume_sessions[(profile,resume_key)]=client
        self.clients[client]=(profile,self.clock());return client

    def client(self, token):
        if token not in self.clients:raise Rejected('client session unknown; re-register without replaying old commands')
        profile,_=self.clients[token];self.clients[token]=(profile,self.clock());return profile

    def command(self, m):
        if m.get('scheduler_instance')!=self.instance:raise Rejected('scheduler instance changed')
        profile=self.client(m['client_id'])
        end=m['operation']=='end'
        if m['operation'] not in ('move','end'):raise Rejected('unknown command operation')
        if not end and not self.ready:raise Rejected('driver not ready or scheduler not bound')
        expected={'manual':'chassis','navigation':'chassis.nav_sdk','gamepad':'chassis.gamepad_sdk'}[profile]
        if not end and m['kind']!=expected:raise Rejected('command kind does not match registered profile')
        ttl_max=1. if profile in ('manual','navigation') else .3
        if not math.isfinite(m['ttl_s']) or not 0<m['ttl_s']<=ttl_max:raise Rejected('source lease exceeded')
        sdk=list(m['sdk'])
        if len(sdk)!=3 or not all(type(v) in (int,float) and math.isfinite(v) for v in sdk):raise Rejected('invalid SDK values')
        linear,yaw=(self.body['max_linear_m_s'],self.body['max_angular_rad_s']) if profile=='manual' else ((.8,.8) if profile=='navigation' else (2.,2.))
        if math.hypot(*sdk[:2])>linear+1e-12 or abs(sdk[2])>yaw+1e-12:raise Rejected('SDK limit exceeded')
        if profile=='manual' and any(abs(sdk[i])>self.body.get('manual_sdk_axes',{}).get(k,linear)+1e-12 for i,k in enumerate(('x','y'))):raise Rejected('manual axis limit exceeded')
        data={k:m[k] for k in ('source_id','task_id','generation','seq','issued_at','ttl_s')}
        data['type']='chassis.end_output' if end else m['kind']
        return self.scheduler.submit(data,m['client_id'],None if end else sdk,
              maintenance=self.maintenance_token if self.maintenance_client==m['client_id'] else None)

    def reserve(self, client):
        self.client(client)
        token=self.scheduler.reserve_maintenance()
        self.maintenance_client=client;self.maintenance_token=token

    def release(self, client):
        self.client(client)
        if client!=self.maintenance_client:raise Rejected('not maintenance owner')
        self.scheduler.release_maintenance(self.maintenance_token)
        self.maintenance_client=None;self.maintenance_token=None
        self.clients.pop(client,None)

    def update_driver(self, instance, calibrated, applied):
        if self.driver and self.driver[0]!=instance:
            self.scheduler.stop('driver_restarted');self.bound_instance=''
        self.driver=(instance,calibrated,self.clock());self.applied=applied

    def tick(self):
        self.scheduler.tick(feedback_fresh=self.ready)
        if self.maintenance_client and self.clock()-self.clients[self.maintenance_client][1]>=2.:
            self.release(self.maintenance_client)
