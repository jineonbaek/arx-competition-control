"""One short, bounded MPPI trial on an audited straight path."""
import argparse
import fcntl
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path as NavPath
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.time import Time
from std_msgs.msg import String, Bool
from axis_trial import AxisTrial, body_delta
from continuous_track import swept_poses
from calibration_gate import wait_ready
from trial_policy import speed_factors,verified_arrival,target_pose,trial_envelope,trial_progress,ReadinessHold
from session_policy import slew_effort,OwnedGoals,valid_effort

p=argparse.ArgumentParser()
p.add_argument('report',type=Path)
p.add_argument('--execute',action='store_true')
p.add_argument('--forward',type=float,default=-.22)
p.add_argument('--left',type=float,default=0.)
p.add_argument('--yaw',type=float,default=0.)
args=p.parse_args()
relative=np.array([args.forward,args.left,args.yaw])
if (not np.isfinite(relative).all() or np.linalg.norm(relative[:2])>.25 or
        abs(relative[2])>.35 or max(abs(relative))<.01):p.error('trial target outside commissioning bounds')
rclpy.init(args=['--ros-args','-r','__node:=arx_mppi_closed_trial',
    '-r','/pb/plan_goal:=/arx_mppi_trial/unused_goal',
    '-r','/pb/dock_goal:=/arx_mppi_trial/unused_dock',
    '-r','/pb/path:=/arx_mppi_trial/unused_path',
    '-r','/clicked_point:=/arx_mppi_trial/unused_point',
    '-r','/pb/chassis_preview:=/arx_mppi_trial/unused_chassis'])
n=AxisTrial()
n.destroy_timer(next(iter(n.timers)))
n.transport.sdk_units=True
n.transport.native_sdk=True
client=ActionClient(n,FollowPath,'/arx_mppi_trial/follow_path')
received={}
def take(key,msg):
    received[key]=(time.monotonic(),msg)
for key,cls,topic in [('sdk',TwistStamped,'/arx_mppi_trial/sdk_preview'),
                      ('status',String,'/arx_mppi_trial/preview_status'),
                      ('map',Bool,'/pb/dual_origin/ready'),
                      ('inputs',Bool,'/arx_mppi_trial/inputs_ready')]:
    n.create_subscription(cls,topic,lambda m,k=key:take(k,m),1)
report=dict(execute=args.execute, target_relative=relative.tolist(), samples=[], commands=0)
handle=None
lock=None
owned=OwnedGoals()
sent_goal=None
class InputGap(RuntimeError):pass
def wait_future(f,limit):
    end=time.monotonic()+limit
    while not f.done() and time.monotonic()<end:n.spin(.03)
    if not f.done():raise RuntimeError('action response timeout')
    return f.result()
def console_idle():
    with n.transport.client.open(n.transport.origin+'/api/state',timeout=.25) as response:
        s=json.load(response)
    return s['execution_enabled'] and s['nav_sdk_ready'] and s['body']['fresh'] and not s['chassis_active'] and not s['capabilities']['chassis']
def current():
    if stale:=n.stale_inputs():raise InputGap('stale '+str(stale))
    when,ok=received.get('map',(0,None))
    if not ok or not ok.data or time.monotonic()-when>.8:raise InputGap('mapping not fresh')
    when,ok=received.get('inputs',(0,None))
    if not ok or not ok.data or time.monotonic()-when>.15:raise InputGap('MPPI inputs not fresh')
    return {frame:n.pose(frame,n.now()) for frame in ('map','odom')}
def pose_msg(x,y,yaw,header):
    m=PoseStamped()
    m.header=header
    m.pose.position.x,m.pose.position.y=float(x),float(y)
    m.pose.orientation.z=math.sin(yaw/2)
    m.pose.orientation.w=math.cos(yaw/2)
    return m

try:
    if args.execute:
        lock=open('/tmp/arx-pb-goal-drive.lock','a')
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    n.spin(3.)
    wait_ready(n,timeout=12.)
    poses=current()
    origin=poses['odom']
    if not console_idle():raise RuntimeError('console not idle or disabled')
    n.spin(.45)
    drift=body_delta(origin,current()['odom'])
    if np.linalg.norm(drift[:2])>.008 or abs(drift[2])>.025:raise RuntimeError('base not stationary')
    for frame,start in poses.items():
        audited=relative.copy()
        distance=np.linalg.norm(relative[:2])
        if distance>.01:audited[:2]*=(distance+.12)/distance
        elif relative[2]:audited[2]+=.10*np.sign(relative[2])
        end=np.array(target_pose(start,audited))
        issue=n.grids[frame][1].along(np.linspace(start,end,15))['first']
        if issue:raise RuntimeError(f'{frame} path blocked: {issue}')
    report['start_odom']=origin.tolist()
    report['start_map']=poses['map'].tolist()
    report['clearance_extra_m']=.12
    if args.execute:
        if not client.wait_for_server(timeout_sec=4.):raise RuntimeError('MPPI unavailable')
        path=NavPath()
        path.header.frame_id='odom'
        path.header.stamp=n.get_clock().now().to_msg()
        for fraction in np.linspace(0.,1.,31):
            path.poses.append(pose_msg(*target_pose(origin,relative*fraction),path.header))
        goal=FollowPath.Goal(path=path,controller_id='FollowPath',goal_checker_id='goal_checker',progress_checker_id='progress_checker')
        sent_goal=client.send_goal_async(goal)
        sent_goal.add_done_callback(lambda future:owned.remember(future.result()))
        handle=wait_future(sent_goal,4.)
        if not handle.accepted:raise RuntimeError('MPPI rejected trial path')
        outcome=handle.get_result_async()
        t=time.monotonic()
        first_motion=None
        reason='time_limit'
        prior=np.zeros(3)
        measured=np.zeros(3)
        measure_pose=origin
        measure_at=t
        last_send=t
        consumed=t
        hold=ReadinessHold()
        while time.monotonic()-t<9.:
            n.spin(.052)
            now=time.monotonic()
            try:poses=current()
            except InputGap:
                n.brake()
                prior=np.zeros(3)
                hold.update(False,now)
                continue
            when,msg=received.get('sdk',(0,None))
            st,status=received.get('status',(0,None))
            preview_ready=msg is not None and status is not None and status.data=='preview_ready' and now-min(when,st)<=.15
            if not preview_ready:
                n.brake();prior=np.zeros(3)
                if first_motion is not None:hold.update(False,now)
                elif now-t>1.:raise RuntimeError('MPPI preview data stale at startup')
                continue
            hold.update(True,now)
            delta=body_delta(origin,poses['odom'])
            if not trial_envelope(delta,relative):
                raise RuntimeError('trial envelope exceeded')
            if now-measure_at>=.25:
                measured=body_delta(measure_pose,poses['odom'])/(now-measure_at)
                measure_pose,measure_at=poses['odom'],now
                # Speed excursions are regulated without abandoning the action/goal.
            if outcome.done():
                res=outcome.result()
                report['action']=[res.status,res.result.error_code]
                reason='arrived' if res.status==4 and res.result.error_code==0 else 'mppi_aborted'
                break
            if when<=consumed:
                if n.active and now-last_send>.14:raise RuntimeError('SDK preview stopped updating')
                continue
            consumed=when
            v=msg.twist
            cmd=np.array([v.linear.x,v.linear.y,v.angular.z])
            xy_factor,yaw_factor=speed_factors(measured)
            cmd[:2]*=xy_factor;cmd[2]*=yaw_factor
            if not valid_effort(cmd):
                raise RuntimeError('Invalid SDK preview command')
            if np.linalg.norm(relative[:2])>.01 and cmd[:2]@relative[:2]<-.006:
                raise RuntimeError('Unexpected opposite effort on trial')
            cmd=np.array(slew_effort(prior,cmd,now-last_send))
            for frame,pose in poses.items():
                for sweep in swept_poses(pose,cmd,measured):
                    if issue:=n.grids[frame][1].along(sweep)['first']:
                        raise RuntimeError(f'{frame} sweep blocked: {issue}')
            if n.stale_inputs() or time.monotonic()-now>.10:
                raise RuntimeError('late command after footprint check')
            report['samples'].append([now-t,*delta.tolist(),*cmd.tolist()])
            if max(abs(cmd))>1e-6:
                n.active=True
                n.transport.send(cmd)
                report['commands']+=1
                first_motion=first_motion or now
            else:n.brake()
            if now-t>3.8 and not trial_progress(delta,relative):raise RuntimeError('insufficient progress')
            prior,last_send=cmd,now
        report['result']=reason
    else:
        report['result']='preflight_passed'
except (Exception,KeyboardInterrupt) as exc:
    report['result']='stopped'
    report['error']=str(exc)
finally:
    try:
        n.brake()
    except Exception as exc:
        report['stop_error']=str(exc)
    report['cancel_errors']=owned.close()
    if sent_goal is not None and not sent_goal.done():
        try:wait_future(sent_goal,2.)
        except Exception as exc:report['cancel_error']=str(exc)
    n.spin(.1)
    if args.execute and 'start_odom' in report:
        try:
            n.spin(1.0)
            a=n.pose('odom',n.now())
            n.spin(1.0)
            b=n.pose('odom',n.now())
            report['final_delta']=body_delta(np.asarray(report['start_odom']),b).tolist()
            report['settled_drift']=body_delta(a,b).tolist()
            report['console_idle']=console_idle()
            target=np.array(target_pose(report['start_odom'],relative))
            report['final_goal_error']=body_delta(b,target).tolist()
            if report.get('result')=='arrived':
                report['result']='arrived_stopped' if verified_arrival(report.get('action'),report['final_goal_error'],report['settled_drift'],report['console_idle']) else 'arrival_not_verified' 
        except Exception as exc:report['settling_error']=str(exc)
    args.report.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='samples'}),flush=True)
    n.destroy_node()
    rclpy.try_shutdown()
    if lock:lock.close()
