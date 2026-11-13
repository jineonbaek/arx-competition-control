"""Resume only a latched read-only body adapter within the SAME LIO process.

Preserves its recorded odom<-lio_world gauge and all Runtime timestamp guards.
No IMU reinitialization, commands, publishers to hardware, or new origin.
"""
import argparse
import fcntl
import json
import sys
from pathlib import Path
import numpy as np
import rclpy
import runtime

parser=argparse.ArgumentParser()
parser.add_argument('--run',type=Path,required=True)
parser.add_argument('--lio-pid',type=int,required=True)
parser.add_argument('--operator-confirmed-static',action='store_true')
args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
proc=Path('/proc')/str(args.lio_pid)
cmd=proc.joinpath('cmdline').read_bytes()
if b'fastlio_mapping' not in cmd:raise RuntimeError('Expected existing LIO process')
identity=proc.joinpath('stat').read_text().rsplit(')',1)[1].split()[19]
config=json.loads((args.run/'geometry.json').read_text())
saved=json.loads((args.run/'world-gauge.json').read_text())
if saved['configuration']!=config['bottom']:raise RuntimeError('Gauge/configuration mismatch')
gauge=np.asarray(saved['odom_from_lio_world'],float)
if (gauge.shape!=(4,4) or not np.isfinite(gauge).all() or
        not np.allclose(gauge[3],[0,0,0,1]) or
        not np.allclose(gauge[:3,:3].T@gauge[:3,:3],np.eye(3),atol=1e-6) or
        not np.isclose(np.linalg.det(gauge[:3,:3]),1.,atol=1e-6)):
    raise RuntimeError('Invalid saved rigid gauge')
lease=(args.run/'body-recovery.lock').open('a')
fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
# Runtime calls this once, before its first body output and static TF publication.
# The original matrix is returned; all odometry validation remains unchanged.
runtime.initial_gauge=lambda _initial_pose:gauge.copy()
rclpy.init(args=['--ros-args','-r','__node:=bottom_upright_body_resumed'])
if args.operator_confirmed_static:
    # Explicit operator recovery after a real manual move, not automatic jump recovery.
    import time
    from nav_msgs.msg import Odometry
    audit=rclpy.create_node('body_operator_static_audit');rows=[]
    def receive(m):
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        if not -100_000_000<=audit.get_clock().now().nanoseconds-stamp<=300_000_000:return
        po=m.pose.pose;tw=m.twist.twist
        if m.header.frame_id!='lio_world' or m.child_frame_id!='bottom_imu':return
        p,q,v,_,_=runtime.body_state([po.position.x,po.position.y,po.position.z],
            [po.orientation.x,po.orientation.y,po.orientation.z,po.orientation.w],
            np.array([tw.linear.x,tw.linear.y,tw.linear.z,tw.angular.x,tw.angular.y,tw.angular.z]),config['bottom']['imu_origin_m'])
        if rows and not 0<stamp-rows[-1][0]<=250_000_000:rows.clear()
        rows.append((stamp,np.asarray(p),np.asarray(q),np.asarray(v)))
    subscription=audit.create_subscription(Odometry,'/single/odometry_raw',receive,50)
    deadline=time.monotonic()+12
    while time.monotonic()<deadline:
        rclpy.spin_once(audit,timeout_sec=.03)
        if len(rows)>=15 and rows[-1][0]-rows[0][0]>=1_400_000_000:break
    if len(rows)<15:raise RuntimeError('Insufficient fresh stationary samples')
    positions=np.array([r[1] for r in rows]);quaternions=np.array([r[2] for r in rows]);velocities=np.array([r[3] for r in rows])
    if (np.linalg.norm(np.ptp(positions,axis=0))>.02 or
        (2*np.arccos(np.clip(np.abs(quaternions@quaternions[0]),0,1))).max()>.03 or
        np.linalg.norm(velocities[:,:3],axis=1).max()>.05 or np.linalg.norm(velocities[:,3:],axis=1).max()>.10):
        raise RuntimeError('Operator static check did not pass')
    seed={'last_published_ns':rows[-1][0],'position_raw_m':rows[-1][1].tolist(),
          'quaternion_raw_xyzw':rows[-1][2].tolist(),'healthy':False,'seed_kind':'operator_confirmed_static',
          'samples':len(rows),'position_span_m':np.ptp(positions,axis=0).tolist()}
    (args.run/'operator-recovery-seed.json').write_text(json.dumps(seed,indent=2))
    audit.destroy_node()
else:
    seed=json.loads((args.run/'body-output-state.json').read_text())
node=runtime.Runtime('body',config,str(root/'models/lift2s_fixed_rear_20261112/body_voxels.npz'),args.run)
node.guard.begin_recovery(seed['last_published_ns'],seed['position_raw_m'],seed['quaternion_raw_xyzw'])
node.output_state=seed.copy();node.output_status(False)
(args.run/'body-recovery.json').write_text(json.dumps({'lio_pid':args.lio_pid,
    'lio_start_ticks':identity,'preserved_gauge':gauge.tolist(),'commands_sent':0},indent=2))
try:
    while rclpy.ok():
        try:
            if proc.joinpath('stat').read_text().rsplit(')',1)[1].split()[19]!=identity:break
        except FileNotFoundError:break
        rclpy.spin_once(node,timeout_sec=.1)
finally:
    node.destroy_node();rclpy.try_shutdown()
