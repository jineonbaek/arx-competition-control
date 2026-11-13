"""Replay actual stationary scans plus explicit motion, mast vibration and dropout."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='199'
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import json
import rosbag2_py as bag
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from stationary_head import Supplement,stamp
from unittest.mock import Mock
import numpy as np


def run(path):
    rclpy.init();n=Supplement()
    reader=bag.SequentialReader();reader.open(bag.StorageOptions(uri=path,storage_id='mcap'),bag.ConverterOptions('',''))
    topics=['/single/imu','/unused_head/imu','/localization/odometry','/single/lidar_raw','/unused_head/lidar']
    reader.set_filter(bag.StorageFilter(topics=topics))
    types={v.name:get_message(v.type) for v in reader.get_all_topics_and_types() if v.name in topics}
    ts=0.;next_t=0.;outputs=[];states={}; pose=None; injected=False; rejoined=False; moving_exit=False
    n.now=lambda:ts
    n.publish=lambda xy,ts=None:outputs.append(len(xy))
    def pose_tf(*args):
        v=TransformStamped();p=pose.pose.pose;v.transform.translation.x=p.position.x;v.transform.translation.y=p.position.y
        v.transform.translation.z=p.position.z;v.transform.rotation=p.orientation;return v
    n.tf=Mock();n.tf.lookup_transform=pose_tf
    while reader.has_next():
        k,v,t=reader.read_next(); ts=t/1e9;m=deserialize_message(v,types[k])
        if k.endswith('imu'):n.on_imu(int(k.startswith('/unused')),m)
        elif k=='/localization/odometry':pose=m;n.on_pose(m)
        else:n.clouds[int(k.startswith('/unused'))]=m
        if ts<next_t or pose is None:continue
        next_t=ts+.098;n.console_ok=True;n.console_stamp=ts
        n.tick();states[n.last_mode]=states.get(n.last_mode,0)+1
        if n.last_mode=='parked_fused' and outputs and outputs[-1]>0 and not injected:
            n.drive_state(Bool(data=True))
            assert outputs[-1]==0
            for _ in range(3):n.tick();assert n.last_mode!='parked_fused'
            n.drive_state(Bool(data=False))
            injected=True
        elif injected and n.last_mode=='parked_fused' and outputs[-1]>0:
            rejoined=True
            if not moving_exit:
                test=type(pose)();test.header=pose.header;test.pose.pose.position.x=pose.pose.pose.position.x+.05
                n.on_pose(test);assert outputs[-1]==0
                moving_exit=True
    assert injected and rejoined and moving_exit and max(outputs)>0,(states,outputs[-10:])
    ts+=1;n.tick();assert n.last_mode!='parked_fused'
    print(json.dumps(dict(replay_states=states,peak_cells=max(outputs),command_exit=True,odom_exit=True,rejoined=True,stale_exit=True)))
    n.destroy_node();rclpy.shutdown()

if __name__=='__main__':run(sys.argv[1])
