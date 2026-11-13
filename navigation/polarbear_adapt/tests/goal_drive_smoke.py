"""Dry integration: ROS goal -> real Planner.short_omni_path -> Driver -> simulated transport."""
import os
assert os.environ['ROS_DOMAIN_ID']=='199'
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from goal_drive import Driver
import axis_trial
from path_audit import AuditGrid
from plan_only import Planner
from unittest.mock import Mock
import time
axis_trial.WebChassis=lambda: Mock()
rclpy.init()
d=Driver(); planner=Planner()
pos=np.zeros(3)
g=AuditGrid(np.zeros((100,100)),[-2.5,-2.5],.05,d.poly)
d.grids={f:(None,g) for f in ('map','odom')}
d.pose=lambda *args: pos.copy()
d.ready=lambda:True
goal=PoseStamped();goal.header.frame_id='map';goal.pose.orientation.w=1.;goal.pose.position.y=.20
out=d.create_publisher(PoseStamped,'/pb/plan_goal',5)
def dispatch():
    until=time.monotonic()+.25
    while time.monotonic()<until:
        rclpy.spin_once(d,timeout_sec=.005);rclpy.spin_once(planner,timeout_sec=.005)
planner.tick=lambda:None
planner.destroy_timer(next(iter(planner.timers)))
dispatch();out.publish(goal);dispatch();assert d.target is None
goal.header.stamp=d.get_clock().now().to_msg()
out.publish(goal);dispatch();assert d.target is not None and planner.target is not None
assert planner.short_omni_path(pos,d.target,g,dict(target=d.target.tolist()))
dispatch();assert d.route is not None
# Reproduce the measured 10.2 degree mismatch between the Smac end and RViz.
path=NavPath();path.header.frame_id='map';path.header.stamp=d.get_clock().now().to_msg()
import copy
endpoint=copy.deepcopy(goal)
endpoint.pose.orientation.z=np.sin(.17853/2)
endpoint.pose.orientation.w=np.cos(.17853/2)
path.poses=[copy.deepcopy(goal),endpoint]
d.receive(path)
assert d.route is not None and abs(d.route[-2,2]-.17853)<1e-6
assert np.allclose(d.route[-1],d.target)
assert planner.short_omni_path(pos,d.target,g,dict(target=d.target.tolist()))
dispatch()
d.seen=d.seq
moves=[]
def simulated_send(command):
    cmd=np.asarray(command)
    moves.append(cmd.copy())
    pos[:]+=cmd*np.array([.014,.014,.035])
# A transient LIO yaw-rate excursion used to abort this exact goal.
original_pose=d.pose
excursion_start=time.monotonic()
def pose_with_excursion(frame,now):
    pose=original_pose(frame,now)
    if frame=="odom" and .18<time.monotonic()-excursion_start<.70:
        pose[2]+=.23
    return pose
d.pose=pose_with_excursion
# Start with an obsolete path 25 cm from the chassis, then deliver an audited
# replacement for the same arrow while it is waiting. No wheel commands before it.
d.route=np.array([[.25,0.,0.],[.25,.20,0.],d.target.copy()])
spin=d.spin
replanned=[]
def simulate_replan(seconds):
    if d.route is None and not replanned and d.target is not None:
        assert not moves
        replanned.append(d.seq)
        assert planner.short_omni_path(pos,d.target,g,dict(target=d.target.tolist()))
    elif replanned and d.route is not None and d.now()-d.route_time>1.5:
        assert planner.short_omni_path(pos,d.target,g,dict(target=d.target.tolist()))
    spin(seconds)
d.spin=simulate_replan
d.transport.send.side_effect=simulated_send
assert d.run_goal()=='arrived_stopped'
assert planner.target is not None and d.target is not None and d.seq==d.seen
assert replanned==[d.seq]
d.spin=spin
assert len(moves)>=4 and np.allclose(moves[-1],0)
assert all(np.linalg.norm(v)>0 for v in moves[:-1])
d.cancel('arrived_stopped');dispatch();assert planner.target is None
goal.header.stamp=d.get_clock().now().to_msg();out.publish(goal);dispatch()
planner.short_omni_path(pos,d.target,g,dict(target=d.target.tolist()));dispatch()
planner.clear();dispatch();assert d.target is None
assert isinstance(d.transport,Mock)
# Simulate a loaded chassis standing just outside arrival tolerance.
from continuous_track import tracking_command, PERIOD
target=np.array([.05,0.,.10])
cmd=np.zeros(3)
for k in range(40):
    cmd,arrived=tracking_command(np.array([np.zeros(3),target]),np.zeros(3),target,cmd,PERIOD,k*PERIOD)
assert not arrived and np.allclose(cmd,[.16,0.,.28/2.5])
from continuous_track import SDK_XY_MAX, SDK_YAW_MAX
target=np.array([1.,0.,.4])
route=np.array([np.zeros(3),target])
last=np.array([SDK_XY_MAX,0.,SDK_YAW_MAX])
from continuous_track import CruiseLimiter
limiter=CruiseLimiter()
initial=limiter.xy
for _ in range(30):limiter.update(np.array([.45,0.,-.50]),last,PERIOD)
slow,done=tracking_command(route,np.zeros(3),target,last,PERIOD,limiter=limiter)
assert not done and 0 < limiter.xy < initial and slow[0]<last[0]
before=limiter.xy
for _ in range(50):limiter.update(np.zeros(3),last,PERIOD)
assert limiter.xy>before
assert planner.short_omni_path(np.zeros(3),np.array([0.,0.,2.5]),g,dict(target=[0.,0.,2.5]))
spin_route=np.array([[0.,0.,0.],[0.,0.,.5],[0.,0.,1.2],[.15,0.,1.2]])
cmd,done=tracking_command(spin_route,np.array([.015,-.012,.20]),spin_route[-1],np.zeros(3),PERIOD)
assert not done and np.allclose(cmd[:2],0) and cmd[2]>0
from continuous_track import projected_progress,OffPath
assert np.allclose(projected_progress(np.array([[0.,0.,0.],[1.,0.,0.]]),np.array([.5,.01,2.]))[1][:2],[.5,0.])
print('PASS: overspeed keeps same goal and plan; bounded breakaway; stale arrow, ROS arrow->audited plan->simulated lateral arrival, finish clear, withdrawal; zero hardware writes')
d.destroy_node();planner.destroy_node();rclpy.shutdown()
