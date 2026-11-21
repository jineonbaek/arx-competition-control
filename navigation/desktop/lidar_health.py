"""Read-only raw lidar/IMU check; never creates LIO, maps or command publishers."""
import argparse,json,math,time
from pathlib import Path

def main():
 import rclpy
 from rclpy.qos import qos_profile_sensor_data
 from sensor_msgs.msg import Imu
 from livox_ros_driver2.msg import CustomMsg
 p=argparse.ArgumentParser();p.add_argument('--timeout',type=float,default=4.);p.add_argument('--stage',choices=['raw','lio'],default='raw');a=p.parse_args()
 root=Path(__file__).resolve().parents[1]
 from lidar_ingress import state
 flags=state()
 if any(type(flags.get(k)) is not bool for k in ['bottom','head']):raise ValueError('雷达开关配置无效')
 rclpy.init();node=rclpy.create_node('arx_raw_lidar_self_check');samples={};topics={'/single/imu':Imu}
 if a.stage=='lio':
  from nav_msgs.msg import Odometry
  from tf2_ros import Buffer,TransformListener
  tf=Buffer();listener=TransformListener(tf,node)
  topics['/localization/odometry']=Odometry
 if flags['bottom']:topics['/single/lidar_raw']=CustomMsg
 if flags['head']:topics.update({'/unused_head/lidar':CustomMsg,'/unused_head/imu':Imu})
 def receive(topic,m):
  stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
  if not math.isfinite(stamp) or (hasattr(m,'point_num') and m.point_num<=0):return
  last,count,received=samples.get(topic,(0.,0,0.))
  if stamp>last:samples[topic]=(stamp,count+1,time.monotonic())
 for topic,cls in topics.items():node.create_subscription(cls,topic,lambda m,t=topic:receive(t,m),qos_profile_sensor_data)
 missing=list(topics);deadline=time.monotonic()+a.timeout
 try:
  while time.monotonic()<deadline:
   rclpy.spin_once(node,timeout_sec=.05);now=node.get_clock().now().nanoseconds/1e9
   missing=[t for t in topics if (s:=samples.get(t,(0,0,0)))[1]<3 or time.monotonic()-s[2]>1. or not -.3<=now-s[0]<=1. or node.count_publishers(t)!=1]
   if not missing and a.stage=='lio':
    try:
     transform=tf.lookup_transform('odom','base_link',rclpy.time.Time())
     stamp=transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
     if not -.3<=now-stamp<=1.:missing.append('odom→base_link TF过期')
    except Exception:missing.append('odom→base_link TF缺失')
   if not missing:break
  print(json.dumps(dict(ready=not missing,missing=missing,enabled=flags,counts={t:s[1] for t,s in samples.items()}),ensure_ascii=False))
  return int(bool(missing))
 finally:
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
