"""Capture a fresh stationary odometry window. Never submits motion commands."""
import json,math,time,sys
from pathlib import Path
import numpy as np

def angle(v):return math.atan2(math.sin(v),math.cos(v))
def alignment(pose):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    rot=np.array([[c,s],[-s,c]]);return rot,-rot@np.array([x,y])
def stationary(samples):
    if len(samples)<10 or samples[-1][0]-samples[0][0]<1.:return False
    poses=np.asarray([p for _,p in samples]);times=np.asarray([t for t,_ in samples])
    return bool(np.isfinite(poses).all() and np.all(np.diff(times)>0) and np.max(np.diff(times))<.3 and
                np.linalg.norm(np.ptp(poses[:,:2],axis=0))<.01 and
                max(abs(angle(p[2]-poses[0,2])) for p in poses)<.015)

def main(destination):
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer,TransformListener,TransformException
    from close_navigation import Backend
    root=Path(__file__).resolve().parent.parent;backend=Backend(root)
    if not backend.idle():raise ValueError('请结束当前任务并停车后设定方向')
    rclpy.init();node=rclpy.create_node('capture_mapping_direction');buf=Buffer();listener=TransformListener(buf,node);samples=[]
    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            try:tf=buf.lookup_transform('odom','base_link',Time())
            except TransformException:continue
            stamp=Time.from_msg(tf.header.stamp).nanoseconds;age=(node.get_clock().now().nanoseconds-stamp)*1e-9
            if not 0<=age<=.5:samples=[];continue
            q=tf.transform.rotation;p=tf.transform.translation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            if not samples or stamp*1e-9>samples[-1][0]:samples.append((stamp*1e-9,[p.x,p.y,yaw]))
            samples=[s for s in samples if samples[-1][0]-s[0]<=1.5]
            if stationary(samples) and backend.idle():
                Path(destination).write_text(json.dumps(dict(odom_pose=samples[-1][1],stamp_ns=stamp,stationary=True)))
                return
        raise ValueError('未取得连续静止且新鲜的车位，请保持车身和机构静止后重试')
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main(sys.argv[1])
