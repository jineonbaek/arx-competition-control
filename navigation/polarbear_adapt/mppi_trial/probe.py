"""Send a path exclusively to the MPPI preview action and measure its output."""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

parser = argparse.ArgumentParser()
parser.add_argument('--smac', action='store_true')
parser.add_argument('--forward', type=float, default=.65)
parser.add_argument('--left', type=float, default=0.)
parser.add_argument('--yaw', type=float, default=0.)
parser.add_argument('--seconds', type=float, default=8.)
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('arx_mppi_preview_probe')
buffer = Buffer()
listener = TransformListener(buffer, node)
follow = ActionClient(node, FollowPath, '/arx_mppi_trial/follow_path')
plan = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
lifecycle = node.create_client(GetState, '/arx_mppi_trial/controller_server/get_state')
topic = '/arx_mppi_trial/raw_velocity'
rows = []
handle = None

def record(message):
    twist = message.twist if isinstance(message, TwistStamped) else message
    rows.append([time.monotonic(), twist.linear.x, twist.linear.y, twist.angular.z])

def pump(until, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.03)
        if until():
            return True
    return False

def result(future, timeout=8.):
    if not pump(future.done, timeout):
        raise RuntimeError('Action response timed out')
    return future.result()

def robot_pose():
    try:
        return buffer.lookup_transform('odom','base_link',Time())
    except TransformException:
        return None

def pose(x,y,angle,frame):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z = math.sin(angle/2)
    p.pose.orientation.w = math.cos(angle/2)
    return p

try:
    deadline = time.monotonic() + 20.
    while time.monotonic() < deadline:
        if (lifecycle.service_is_ready() and follow.server_is_ready() and robot_pose() is not None
                and result(lifecycle.call_async(GetState.Request()), 2.).current_state.id
                    == State.PRIMARY_STATE_ACTIVE):
            break
        rclpy.spin_once(node, timeout_sec=.10)
    else:
        raise RuntimeError('Preview controller and robot TF did not become ready')
    assert pump(lambda: bool(dict(node.get_topic_names_and_types()).get(topic)), 5.)
    kind = dict(node.get_topic_names_and_types())[topic]
    assert len(kind) == 1 and kind[0] in ('geometry_msgs/msg/Twist','geometry_msgs/msg/TwistStamped'), kind
    node.create_subscription(TwistStamped if kind[0].endswith('TwistStamped') else Twist, topic, record, 20)
    frame = 'map' if args.smac else 'odom'
    tr = buffer.lookup_transform(frame,'base_link',Time()).transform
    q, p = tr.rotation, tr.translation
    a = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    dx = args.forward*math.cos(a)-args.left*math.sin(a)
    dy = args.forward*math.sin(a)+args.left*math.cos(a)
    path = Path()
    path.header.frame_id = frame
    if args.smac:
        assert pump(plan.server_is_ready, 8.)
        request = ComputePathToPose.Goal()
        request.goal = pose(p.x+dx,p.y+dy,a+args.yaw,frame)
        request.planner_id = 'GridBased'
        accepted = result(plan.send_goal_async(request))
        assert accepted.accepted
        response = result(accepted.get_result_async()).result
        if response.error_code or len(response.path.poses) <= 1:
            raise RuntimeError(f'Smac rejected the preview goal: {response.error_code} {response.error_msg}')
        path = response.path
    else:
        for t in np.linspace(0.,1.,31):
            path.poses.append(pose(p.x+t*dx,p.y+t*dy,a+t*args.yaw,frame))
    request = FollowPath.Goal()
    request.path = path
    request.controller_id = 'FollowPath'
    request.goal_checker_id = 'goal_checker'
    request.progress_checker_id = 'progress_checker'
    handle = result(follow.send_goal_async(request))
    assert handle.accepted
    finished = handle.get_result_async()
    pump(finished.done, args.seconds)
    before_cancel = len(rows)
    if not finished.done():
        result(handle.cancel_goal_async())
    assert pump(finished.done, 4.)
    outcome = finished.result()
    pump(lambda:False,.3)
    speeds = np.asarray(rows)
    assert before_cancel >= 5 and np.isfinite(speeds).all(), (before_cancel,outcome)
    gaps = np.diff(speeds[:before_cancel,0])*1000
    print(json.dumps(dict(path_poses=len(path.poses),status=outcome.status,
        error_code=outcome.result.error_code,error_message=outcome.result.error_msg,
        commands=before_cancel, interval_ms=np.percentile(gaps,[50,95,99]).tolist(),
        max_abs=np.abs(speeds[:,1:]).max(axis=0).tolist(), last=speeds[-1,1:].tolist())))
    assert outcome.status in (4,5) and outcome.result.error_code == 0, outcome
    assert np.abs(speeds[-1,1:]).max()<1e-6
finally:
    if handle is not None and handle.status in (1,2,3):
        pump(handle.cancel_goal_async().done, 2.)
    node.destroy_node()
    rclpy.try_shutdown()
