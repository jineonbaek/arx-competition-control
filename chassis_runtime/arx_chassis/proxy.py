"""Controller facade; all motion arbitration lives in the independent ROS process."""
import hashlib
import time
from .client import Session,shared_transport
from .core import Rejected
class ChassisProxy:
    def __init__(self):
        self.transport=shared_transport();self.sessions={};self.source_sessions={}
        self.maintenance=None;self._maintenance_at=0.;self._heartbeat=None
    def session(self,client,profile='manual'):
        key=(client,profile)
        if key not in self.sessions:
            resume=hashlib.sha256(('web:'+client).encode()).hexdigest() if isinstance(client,str) else ''
            self.sessions[key]=Session(profile,self.transport,resume_key=resume)
        return self.sessions[key]
    @property
    def active(self):
        s=self.transport.state
        if s is None or time.monotonic()-self.transport.state_at>.5 or not s.active:return None
        client=next((k[0] for k,v in self.sessions.items() if v.client_id==s.client_id),s.client_id)
        return dict(client=client,sdk=list(s.sdk),deadline=time.monotonic()+max(0.,s.expires_at-time.time()),output_seq=s.output_seq,identity=(s.source_id,s.task_id,s.generation),source_id=s.source_id)
    def submit(self,m,client,sdk,*,maintenance=None):
        if maintenance is not None and maintenance is not self.maintenance:raise Rejected('maintenance token expired')
        p=dict(m);source=(client,p['source_id'])
        if p['type']=='chassis.end_output':
            session=self.source_sessions.get(source)
            if session is None:
                profile='navigation' if p['source_id'].startswith('navigation-') else 'gamepad' if p['source_id'].startswith('gamepad-') else 'manual'
                session=self.session(client,profile);self.source_sessions[source]=session
        else:
            profile={'chassis.nav_sdk':'navigation','chassis.gamepad_sdk':'gamepad'}.get(p['type'],'manual')
            session=self.session(client,profile);self.source_sessions[source]=session
        if sdk is not None:p.update(zip(('x','y','omega'),sdk))
        return session.send(p)
    def stop(self,reason='explicit_stop'):
        self.session('__operator__').manage('stop');return dict(stopped=True)
    def end_client(self,client):
        for (owner,profile),session in list(self.sessions.items()):
            if owner==client:session.manage('end_client')
    def disconnect(self,client,identity=None):
        # SSE observes a shared cookie; disconnect alone does not own motion.
        if identity is not None:
            source,task,generation=identity;session=self.source_sessions.get((client,source))
            if session:session.send(dict(type='chassis.end_output',source_id=source,task_id=task,generation=generation,seq=1,issued_at=time.time(),ttl_s=1. if session.profile=='manual' else .3))
    def reserve_maintenance(self):
        token=object();self.session(token).manage('reserve');self.maintenance=token;return token
    def release_maintenance(self,token):
        if token is not self.maintenance:raise Rejected('maintenance token expired')
        try:self.session(token).manage('release')
        finally:self.maintenance=None
    def expire(self,seq):
        a=self.active
        if a:
            for (owner,_),session in self.sessions.items():
                if owner==a['client']:session.manage('expire',output_seq=seq)
    def tick(self,**kw):
        # Never block or throw into the web ROS observer's feedback loop.
        if not self.maintenance:return
        if self._heartbeat and not self._heartbeat.done():return
        if time.monotonic()-self._maintenance_at<.5:return
        self._maintenance_at=time.monotonic();token=self.maintenance;session=self.session(token)
        if not self.transport.service.service_is_ready():self.maintenance=None;return
        req=self.transport.Manage.Request();req.operation='heartbeat';req.client_id=session.client_id;req.scheduler_instance=session.instance
        try:
            self._heartbeat=self.transport.service.call_async(req)
            def done(f):
                try:
                    res=f.result()
                    if res:self.transport.receive(res.state)
                    if not res or not res.accepted:
                        if self.maintenance is token:self.maintenance=None
                except Exception:
                    if self.maintenance is token:self.maintenance=None
            self._heartbeat.add_done_callback(done)
        except Exception:self.maintenance=None
    def snapshot(self):
        s=self.transport.state;a=self.active
        if a:a={k:v for k,v in a.items() if k!='client'}
        return dict(policy='last_valid_writer',active=a,output_seq=s.output_seq if s else 0,
                    maintenance=s.maintenance if s else False,backend='ros',
                    scheduler_instance=s.scheduler_instance if s else '',
                    ready=bool(s and s.ready and time.monotonic()-self.transport.state_at<.5))
