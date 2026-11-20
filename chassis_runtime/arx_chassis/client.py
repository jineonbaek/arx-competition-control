"""Typed ROS transport with an independent context in hardware domain 191."""
import os
import atexit
import sys
import threading
import time
import uuid
from pathlib import Path
from .core import Rejected

def load_interfaces():
    # Existing callers may already be running with a different ROS overlay.
    root=Path(__file__).resolve().parents[1]/'install'
    if (root/'arx_chassis_interfaces').exists():root=root/'arx_chassis_interfaces'
    for p in root.glob('lib/python*/site-packages'):
        if str(p) not in sys.path:sys.path.insert(0,str(p))
    import ctypes
    for name in ('libarx_chassis_interfaces__rosidl_generator_c.so','libarx_chassis_interfaces__rosidl_generator_py.so','libarx_chassis_interfaces__rosidl_typesupport_c.so'):
        p=root/'lib'/name
        if p.exists():ctypes.CDLL(str(p),mode=ctypes.RTLD_GLOBAL)
    for p in root.glob('lib/libarx_chassis_interfaces__rosidl_typesupport_*.so'):
        ctypes.CDLL(str(p),mode=ctypes.RTLD_GLOBAL)
    from arx_chassis_interfaces.msg import Command, Result, State
    from arx_chassis_interfaces.srv import Manage
    return Command,Result,State,Manage

class Transport:
    def __init__(self, *, domain=191, namespace='/chassis'):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        self.rclpy=rclpy;self.context=Context();rclpy.init(context=self.context,domain_id=domain)
        self.Command,Result,State,self.Manage=load_interfaces()
        self.node=rclpy.create_node('chassis_client_'+uuid.uuid4().hex[:12],context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.pending={};self.lock=threading.RLock();self.state=None;self.state_at=0.;self.closed=False
        self.pub=self.node.create_publisher(self.Command,namespace+'/command',32)
        self.service=self.node.create_client(self.Manage,namespace+'/manage')
        self.node.create_subscription(Result,namespace+'/command_result',self.result,32)
        self.node.create_subscription(State,namespace+'/state',self.receive,1)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()

    def receive(self,state):
        with self.lock:self.state=state;self.state_at=time.monotonic()

    def result(self,result):
        with self.lock:
            item=self.pending.get(result.request_id)
            if item:item[1].append(result);item[0].set()
            self.state=result.state;self.state_at=time.monotonic()

    def manage(self,operation,*,client_id='',profile='',resume_key='',instance='',output_seq=0,timeout=.8):
        if not self.service.wait_for_service(timeout_sec=timeout):raise Rejected('ROS chassis scheduler unavailable')
        req=self.Manage.Request();req.operation=operation;req.client_id=client_id;req.profile=profile;req.resume_key=resume_key;req.scheduler_instance=instance;req.output_seq=output_seq
        event=threading.Event();future=self.service.call_async(req);future.add_done_callback(lambda f:event.set())
        if not event.wait(timeout):raise Rejected('ROS chassis management timeout')
        res=future.result()
        if res is None:raise Rejected('ROS chassis management failed')
        self.receive(res.state)
        if not res.accepted:raise Rejected(res.reason)
        return res

    def send(self,payload,client_id,instance,*,timeout=.18):
        m=self.Command();m.request_id=uuid.uuid4().hex;m.client_id=client_id;m.scheduler_instance=instance
        m.operation='end' if payload['type']=='chassis.end_output' else 'move';m.kind=payload['type']
        for k in ('source_id','task_id','generation','seq'):setattr(m,k,payload[k])
        m.issued_at=float(payload['issued_at']);m.ttl_s=float(payload['ttl_s'])
        m.sdk=[float(payload.get(k,0.)) for k in ('x','y','omega')]
        event=threading.Event();box=[]
        with self.lock:self.pending[m.request_id]=(event,box)
        try:
            self.pub.publish(m)
            if not event.wait(timeout):raise Rejected('ROS chassis command acknowledgement timeout')
            res=box[0]
            if not res.accepted:raise Rejected(res.reason)
            return dict(ok=True,output_seq=res.output_seq,scheduler_instance=res.state.scheduler_instance,accepted=True)
        finally:
            with self.lock:self.pending.pop(m.request_id,None)

    def close(self):
        if self.closed:return
        self.closed=True;self.executor.shutdown(timeout_sec=2.);self.thread.join(timeout=2.)
        self.node.destroy_node();self.context.shutdown()

_shared=None
_shared_lock=threading.Lock()
def shared_transport():
    global _shared
    with _shared_lock:
        if _shared is None:
            _shared=Transport(domain=int(os.environ.get('ARX_CHASSIS_DOMAIN','191')))
            atexit.register(_shared.close)
        return _shared

class Session:
    def __init__(self,profile,transport=None,*,resume_key=''):
        self.profile=profile;self.resume_key=resume_key;self.transport=transport or shared_transport();self.client_id='';self.instance='';self.last_identity=None;self.fenced_identity=None
    def register(self):
        result=self.transport.manage('register',profile=self.profile,resume_key=self.resume_key,timeout=5.)
        self.client_id=result.client_id;self.instance=result.state.scheduler_instance
    def ensure(self):
        state=self.transport.state
        if not self.client_id or (state and state.scheduler_instance!=self.instance):
            if self.client_id:self.fenced_identity=self.last_identity
            self.register()
    def send(self,payload):
        identity=tuple(payload[k] for k in ('source_id','task_id','generation'))
        self.ensure()
        if payload['type']!='chassis.end_output' and identity==self.fenced_identity:
            raise Rejected('scheduler restarted; release controls and begin a new task')
        self.last_identity=identity
        return self.transport.send(payload,self.client_id,self.instance)
    def manage(self,operation,**kw):
        self.ensure();return self.transport.manage(operation,client_id=self.client_id,instance=self.instance,**kw)
