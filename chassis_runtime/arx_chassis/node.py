"""Independent ROS chassis arbiter. Run with python -m arx_chassis.node."""
import argparse
import fcntl
import json
import os
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from arx_chassis_interfaces.msg import Command, Result, State, Output
from arx_chassis_interfaces.srv import Manage
from .core import Core, Rejected

class SchedulerNode(Node):
    def __init__(self, config, shadow=False, namespace='/chassis'):
        super().__init__('arx_chassis_scheduler')
        self.shadow=shadow;self.namespace=namespace;self.bind_future=None;self.bind_at=0.
        self.output_pub=self.create_publisher(Output,namespace+'/ordered_output',QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE))
        self.result_pub=self.create_publisher(Result,namespace+'/command_result',32)
        self.state_pub=self.create_publisher(State,namespace+'/state',1)
        self.core=Core(self.emit,body=config['body'])
        self.config=config
        self.create_subscription(Command,namespace+'/command',self.command,32)
        self.create_service(Manage,namespace+'/manage',self.manage)
        self.create_subscription(String,'/web/chassis/output_status',self.driver,1)
        self.bind_client=self.create_client(SetParameters,'/lift_controller/set_parameters')
        self.create_timer(.02,self.tick)
        self.get_logger().info('scheduler instance '+self.core.instance+(' SHADOW' if shadow else ''))

    def enabled(self):
        b=self.config['body']
        return self.config['execution_enabled'] and b.get('chassis_verified') is True and b.get('chassis_stop_verified') is True

    def state(self):
        c=self.core;s=State();s.scheduler_instance=c.instance;s.ready=c.ready and self.enabled()
        s.driver_instance=c.driver[0] if c.driver else '';a=c.scheduler.active
        s.active=a is not None;s.output_seq=c.scheduler.output_seq;s.maintenance=c.maintenance_client is not None
        if a:
            s.client_id=a['client'];s.source_id=a['source_id'];s.task_id=a['identity'][1];s.generation=a['identity'][2]
            s.sdk=[float(x) for x in a['sdk']];s.expires_at=c.wall()+max(0.,a['deadline']-c.clock())
        s.applied_sdk=[float(x) for x in c.applied.get('applied_sdk',[0.,0.,0.])]
        s.applied_seq=c.applied.get('output_seq',0)
        return s

    def emit(self, output):
        c=self.core
        if self.shadow:return
        if not c.driver or c.bound_instance!=c.instance:
            if any(output['sdk']):raise Rejected('no bound driver')
            return
        m=Output();m.scheduler_instance=c.instance;m.driver_instance=c.driver[0]
        for k in ('output_seq','issued_at','ttl_s','reason'):setattr(m,k,output[k])
        m.sdk=[float(x) for x in output['sdk']];m.source_id=output.get('source_id','')
        self.output_pub.publish(m)

    def driver(self,m):
        try:
            d=json.loads(m.data)
            if d.get('ordered_output') is not True or len(d.get('applied_sdk',[]))!=3:return
            self.core.update_driver(d['driver_instance'],d['calibrated'],d)
            self.core.bound_instance=d.get('scheduler_instance','')
        except (ValueError,KeyError,TypeError):return

    def command(self,m):
        result=Result();result.request_id=m.request_id
        try:
            self.core.output_connected=self.output_pub.get_subscription_count()>0
            if not self.enabled() and m.operation=='move':raise Rejected('chassis execution disabled or unverified')
            d={k:getattr(m,k) for k in m.get_fields_and_field_types()};d['sdk']=[float(x) for x in m.sdk]
            age_ms=(time.time()-m.issued_at)*1000.
            if m.operation=='move' and age_ms>=100.:
                self.get_logger().warning('chassis_delivery_delay age_ms=%.1f source=%s generation=%d seq=%d' %
                    (age_ms,m.source_id,m.generation,m.seq),throttle_duration_sec=3.)
            out=self.core.command(d);result.accepted=True;result.output_seq=out.get('output_seq',0)
        except (Rejected,KeyError,TypeError,OverflowError) as e:result.reason=str(e)
        result.state=self.state();self.result_pub.publish(result)

    def manage(self,req,res):
        try:
            if req.operation=='register':res.client_id=self.core.register(req.profile,req.resume_key)
            else:
                if req.scheduler_instance!=self.core.instance:raise Rejected('scheduler instance changed')
                self.core.client(req.client_id)
                if req.operation=='stop':self.core.scheduler.stop('explicit_stop')
                elif req.operation=='end_client':self.core.scheduler.end_client(req.client_id)
                elif req.operation=='reserve':self.core.reserve(req.client_id)
                elif req.operation=='release':self.core.release(req.client_id)
                elif req.operation=='expire':
                    a=self.core.scheduler.active
                    if a and a['client']==req.client_id:self.core.scheduler.expire(req.output_seq)
                elif req.operation!='heartbeat':raise Rejected('unknown management operation')
            res.accepted=True
        except (Rejected,TypeError,KeyError) as e:res.reason=str(e)
        res.state=self.state();return res

    def tick(self):
        try:self.core.tick()
        except (Rejected,RuntimeError) as e:self.get_logger().error(str(e))
        c=self.core
        c.output_connected=self.output_pub.get_subscription_count()>0
        # Bind only to an idle driver. Driver validates mode/zero/deadline itself.
        if not self.shadow and c.driver and c.bound_instance!=c.instance and time.monotonic()-self.bind_at>.5:
            if self.bind_future is None or self.bind_future.done():
                if self.bind_client.service_is_ready():
                    request=SetParameters.Request();request.parameters=[Parameter(name='chassis_scheduler_instance',value=ParameterValue(type=ParameterType.PARAMETER_STRING,string_value=c.instance))]
                    self.bind_future=self.bind_client.call_async(request);self.bind_at=time.monotonic()
        self.state_pub.publish(self.state())

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',required=True);parser.add_argument('--shadow',action='store_true');parser.add_argument('--namespace',default='/chassis')
    args=parser.parse_args();domain=int(os.environ.get('ROS_DOMAIN_ID','191'))
    if args.shadow and args.namespace=='/chassis':args.namespace='/chassis_shadow'
    if not args.shadow and args.namespace!='/chassis':raise ValueError('Live scheduler namespace must be /chassis')
    lock=open('/tmp/arx-chassis-scheduler-'+str(domain)+('-shadow-'+args.namespace.replace('/','_') if args.shadow else '-hardware')+'.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rclpy.init();node=SchedulerNode(json.loads(Path(args.config).read_text()),args.shadow,args.namespace)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        # End only this process's currently active generation; restart begins empty.
        try:node.core.scheduler.stop('scheduler_shutdown')
        except Exception:pass
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
