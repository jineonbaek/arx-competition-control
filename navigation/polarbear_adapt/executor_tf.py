"""TF-only reception isolated from navigation, HTTP and geometry callbacks."""
import os,threading,time
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor,ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy,HistoryPolicy,ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TFReceiver:
    def __init__(self,buffer,context,*,use_sim_time=False):
        self.buffer=buffer;self.lock=threading.Lock();self.edges={};self.errors=[];self.closed=False
        self.node=Node(f'exec_tf_rx_{os.getpid()}_{id(self):x}',namespace='/_pb_internal',
            context=context,use_global_arguments=False,enable_rosout=False,
            start_parameter_services=False,parameter_overrides=[Parameter('use_sim_time',value=use_sim_time)])
        dynamic=QoSProfile(depth=10,history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,reliability=ReliabilityPolicy.RELIABLE)
        static=QoSProfile(depth=100,history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.sub=self.node.create_subscription(TFMessage,'/tf',self.dynamic,dynamic)
        self.static_sub=self.node.create_subscription(TFMessage,'/tf_static',self.static,static)
        self.executor=SingleThreadedExecutor(context=context);self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.spin,name='executor-tf-reception',daemon=True);self.thread.start()
    def spin(self):
        try:self.executor.spin()
        except ExternalShutdownException:pass
        except Exception as exc:
            with self.lock:self.errors.append(str(exc));self.errors=self.errors[-5:]
    def dynamic(self,message,info):self.receive(message,info,False)
    def static(self,message,info):self.receive(message,info,True)
    def receive(self,message,info,static):
        receipt=time.monotonic();now=self.node.get_clock().now().nanoseconds
        received=getattr(info,'received_timestamp',0);queue_ms=(time.time_ns()-received)/1e6 if received else None
        if queue_ms is not None and not 0<=queue_ms<60000:queue_ms=None
        for transform in message.transforms:
            start=time.monotonic();key=transform.header.frame_id+'->'+transform.child_frame_id
            stamp=transform.header.stamp.sec*1000000000+transform.header.stamp.nanosec
            try:
                if static:self.buffer.set_transform_static(transform,'executor_tf_receiver')
                else:self.buffer.set_transform(transform,'executor_tf_receiver')
            except Exception as exc:
                with self.lock:self.errors.append(key+': '+str(exc));self.errors=self.errors[-5:]
            elapsed=(time.monotonic()-start)*1000
            with self.lock:
                row=self.edges.setdefault(key,dict(count=0,advances=0,latest_source_ns=-1,max_callback_ms=0.))
                row['count']+=1;row['receipt']=receipt;row['static']=static
                if stamp>row['latest_source_ns']:row['advances']+=1;row['latest_source_ns']=stamp
                row.update(source_age_at_receipt_ms=None if static else (now-stamp)/1e6,
                    callback_ms=elapsed,queue_ms=queue_ms,max_callback_ms=max(row['max_callback_ms'],elapsed))
    def diagnostic(self):
        now=time.monotonic()
        with self.lock:
            edges={k:{key:value for key,value in v.items() if key!='receipt'} | {'receipt_gap_ms':(now-v['receipt'])*1000}
                for k,v in self.edges.items() if k.endswith('->base_link') or k=='map->odom'}
            return dict(receiver_alive=self.thread.is_alive(),dynamic_depth=10,static_depth=100,
                edges=edges,errors=list(self.errors))
    def close(self):
        if self.closed:return
        self.closed=True;self.executor.shutdown(timeout_sec=2.);self.thread.join(2.)
        if self.thread.is_alive():raise RuntimeError('TF receiver thread did not stop')
        self.executor.remove_node(self.node);self.node.destroy_node()
