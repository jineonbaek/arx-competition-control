"""Relay existing route topics through our domain-192 observer; never drives hardware."""
import copy
import json
import time
from pathlib import Path
from tempfile import NamedTemporaryFile


def atomic_json(path,value):
    # The standalone ROS observer does not import the web/agent package environment.
    with NamedTemporaryFile(mode='w',encoding='utf-8',dir=path.parent,delete=False) as out:
        temp=Path(out.name)
        try:json.dump(value,out,ensure_ascii=False,allow_nan=False)
        except BaseException:
            temp.unlink(missing_ok=True);raise
    try:temp.replace(path)
    finally:temp.unlink(missing_ok=True)


def read(path):
    try:return json.loads(path.read_text())
    except (OSError,ValueError):return {}


class RouteMailbox:
    def __init__(self, root, snapshot):
        self.root,self.snapshot=Path(root),snapshot

    def command(self, action, request_id, route_id=None):
        route=self.snapshot().get('route') or {}
        if not route.get('available'):raise ValueError('导航执行端或地图观察服务未就绪；请先启动导航并准备路线')
        value=dict(action=action,request_id=request_id,route_id=route_id)
        path=self.root/'navigation-command.json'
        old=read(path)
        if old.get('request_id')==request_id:
            if old!=value:raise ValueError('路线请求编号不能改内容')
            return
        self.root.mkdir(parents=True,exist_ok=True)
        atomic_json(path,value)

    def receipt(self, request_id):
        value=read(self.root/'navigation-command-result.json')
        return value if value.get('request_id')==request_id else None


class RouteRelay:
    def __init__(self, root):
        self.root=Path(root)
        # A restarted observer must never replay a queued movement request.
        self.seen=read(self.root/'navigation-command.json').get('request_id')
        self.definition={};self.status='';self.drive_status='';self.drive_received=0.;self.publishers={}

    def attach(self,node):
        from std_msgs.msg import Empty,String
        from rclpy.qos import QoSProfile,DurabilityPolicy
        self.empty=Empty
        self.publishers={kind:node.create_publisher(Empty,'/pb/waypoints/'+kind,5) for kind in ('start','clear')}
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String,'/pb/waypoints/definition',self.on_definition,qos)
        node.create_subscription(String,'/pb/waypoints/status',lambda m:setattr(self,'status',m.data),qos)
        node.create_subscription(String,'/pb/drive_status',self.on_drive,10)
        node.create_timer(.1,self.tick)

    def on_definition(self,msg):
        try:
            value=json.loads(msg.data)
            if isinstance(value,dict) and isinstance(value.get('points'),list):self.definition=value
        except ValueError:pass

    def available(self):
        return bool(self.publishers) and all(p.get_subscription_count()>0 for p in self.publishers.values())

    def on_drive(self,msg):self.drive_status=msg.data;self.drive_received=time.monotonic()

    def publish(self,action):self.publishers[action].publish(self.empty())

    def snapshot(self):
        return copy.deepcopy(dict(available=self.available(),definition=self.definition,
                                  status=self.status,drive_status=self.drive_status,drive_received=self.drive_received))

    def tick(self):
        value=read(self.root/'navigation-command.json');request=value.get('request_id')
        if not request or request==self.seen:return
        self.seen=request;result=dict(request_id=request,status='rejected')
        try:
            if set(value)!={'request_id','action','route_id'} or value['action'] not in ('start','clear'):
                raise ValueError('无效路线请求')
            if not self.available():raise ValueError('导航执行端未订阅路线入口')
            if value['action']=='start' and value['route_id']!=self.definition.get('route_id'):
                raise ValueError('路线已被修改，请重新核对后开始')
            self.publish(value['action']);result['status']='published'
        except Exception as error:result['reason']=str(error)[:300]
        atomic_json(self.root/'navigation-command-result.json',result)
