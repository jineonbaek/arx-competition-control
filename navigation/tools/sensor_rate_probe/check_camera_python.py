#!/usr/bin/env python3
"""Reliable Python subscriber benchmark; stores counters only, no image payload."""
import json,time,sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
from sensor_msgs.msg import Image
rclpy.init();node=Node('python_camera_rate_probe');stats={};active=False;subs=[]
def cb(topic,msg):
 if not active:return
 now=time.monotonic();s=stats.setdefault(topic,{'count':0,'first':now,'last':now});s['count']+=1;s['last']=now
for topic in ('/sensors/d435/color/image_raw','/sensors/d435/depth/image_rect_raw'):
 subs.append(node.create_subscription(Image,topic,lambda msg,t=topic:cb(t,msg),QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST)))
end=time.monotonic()+3
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
active=True;end=time.monotonic()+20
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
result={t:{'messages':s['count'],'hz':(s['count']-1)/(s['last']-s['first'])} for t,s in stats.items() if s['count']>1}
Path(sys.argv[1]).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));node.destroy_node();rclpy.shutdown()
