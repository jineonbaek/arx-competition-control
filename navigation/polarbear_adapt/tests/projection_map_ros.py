"""Isolated ROS fixture, including the Nav2 PGM saver. Set ROS_DOMAIN_ID=179."""
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster
from live_static_map import OnlineMap

assert os.environ['ROS_DOMAIN_ID']=='179'
report=Path(sys.argv[1])
rclpy.init()
node=OnlineMap()
probe=rclpy.create_node('projection_fixture')
executor=SingleThreadedExecutor()
executor.add_node(node)
executor.add_node(probe)
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
messages={}
for topic in ('map','persistent_map'):
    probe.create_subscription(OccupancyGrid,'/pb/'+topic,lambda m,k=topic: messages.update({k:m}),q)
ref=probe.create_publisher(OccupancyGrid,'/pb/reference_map',q)
reset=probe.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
ack=probe.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',10)
tf=TransformBroadcaster(probe)
def pump(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        executor.spin_once(timeout_sec=.01)
def send(x,y,yaw,alignment=0.):
    stamp=probe.get_clock().now().to_msg()
    batch=[]
    for parent,child,px,py,a in [('map','odom',alignment,0.,0.),('odom','base_link',x,y,yaw)]:
        m=TransformStamped()
        m.header.frame_id=parent;m.child_frame_id=child;m.header.stamp=stamp
        m.transform.translation.x=float(px);m.transform.translation.y=float(py)
        m.transform.rotation.w=math.cos(a/2);m.transform.rotation.z=math.sin(a/2)
        batch.append(m)
    tf.sendTransform(batch)
def update(*args):
    for _ in range(4):
        send(*args)
        pump(.045)
def latest(topic='map'):
    return np.asarray(messages[topic].data,dtype=np.int8).reshape(80,80)
def overlay():
    a=latest();base=latest('persistent_map')
    assert np.array_equal(base,original)
    diff=a!=base
    assert np.all(base[diff]==-1) and np.all(a[diff]==0)
    return set(np.flatnonzero(diff))
def expected(x,y,yaw):
    return set(node.projection.unknown_cells(original,(-2,-2),float(grid.info.resolution),(x,y,yaw)))
grid=OccupancyGrid();grid.header.frame_id='map'
grid.info.width=grid.info.height=80;grid.info.resolution=.05
grid.info.origin.position.x=grid.info.origin.position.y=-2.;grid.info.origin.orientation.w=1.
original=np.full((80,80),-1,dtype=np.int8)
original[40,40]=100;original[40,41]=0
grid.data=original.ravel().tolist()
pump(.35)
ref.publish(grid)
pump(.20)
assert not overlay()
update(0,0,0);initial=overlay();assert initial==expected(0,0,0) and initial
# Hold a fresh pose while map_saver runs, without republishing the reference.
saver=subprocess.Popen(['ros2','run','nav2_map_server','map_saver_cli','-t','/pb/persistent_map','-f',str(report/'fixture-grid'),'--fmt','pgm','--mode','trinary'],stdout=subprocess.DEVNULL)
until=time.monotonic()+15
while saver.poll() is None and time.monotonic()<until:update(0,0,0)
assert saver.wait(timeout=1)==0
image=np.asarray(Image.open(report/'fixture-grid.pgm'))[::-1]
assert np.all(image.flat[list(initial)]==205) and image[40,40]==0 and image[40,41]==254
update(.70,.11,0);moved=overlay();assert moved==expected(.70,.11,0) and not (moved & initial)
update(.70,.11,.88);turned=overlay();assert turned==expected(.70,.11,.88) and moved-turned and turned-moved
pump(.52);assert not overlay()
update(.70,.11,.88);assert overlay()==turned
send(.70,.11,.88,.52);pump(.14);assert not overlay()
update(.70,.11,.88,.52);assert overlay()==expected(1.22,.11,.88)
pose=PoseWithCovarianceStamped();pose.header.frame_id='map';pose.header.stamp=probe.get_clock().now().to_msg()
reset.publish(pose);update(.70,.11,.88,.52);assert not overlay()
pose.header.stamp=probe.get_clock().now().to_msg();ack.publish(pose)
update(.70,.11,.88,.52);assert overlay()==expected(1.22,.11,.88)
print(json.dumps(dict(initial=len(initial),old_cells_restored=len(initial-moved),rotation_restored=len(moved-turned),rotation_new=len(turned-moved),stale_retracted=True,relocalization_retracted=True,exported_unknown=len(initial))))
executor.shutdown();probe.destroy_node();node.destroy_node();rclpy.shutdown()
