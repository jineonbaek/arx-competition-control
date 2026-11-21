"""One bounded body-forward T1 contact attempt before handing off real work."""
import json,time
import numpy as np
import rclpy
from std_msgs.msg import Bool
from mppi_trial.speed_profile import SLOW_SDK
from special_segments import StraightChain

DURATION=2.
COMMAND_TTL=.2
REFRESH_INTERVAL=.05

def point_key(n):
 r=n.waypoints.route;i=r.target_index
 if getattr(r,'t2_index',None) is not None and i==r.t2_index-1:return None
 if not r.running or r.state!='running' or i is None or not 0<=i<len(r.points):return None
 p=r.points[i]
 if p.get('type')!='task' or p.get('point_id')!='point_1':return None
 return (r.route_id,r.mission_id,p['id'],tuple(p['pose']))

def finish_arrival(n,epoch,result):
 if result!='arrived_stopped' or point_key(n) is None:return result
 state=getattr(n,'t1_contact_session',None)
 if state is None or state['epoch']!=epoch:
  state=dict(epoch=epoch,key=point_key(n),attempted=False,complete=False,chain=None,handle=None)
  n.t1_contact_session=state
 return resume_contact(n,epoch)

def resume_contact(n,epoch):
 state=n.t1_contact_session
 def intent():
  n.current_intent(epoch)
  if state['epoch']!=epoch or state['key']!=point_key(n) or n.waypoints.blocking():raise ValueError('goal_changed')
 def proof(require_idle=False):
  intent()
  with n.transport.client.open(n.transport.origin+'/api/state',timeout=.3) as reply:s=json.load(reply)
  if (not s['execution_enabled'] or not s['body']['fresh'] or not s.get('nav_sdk_ready',False)
      or s['capabilities']['chassis'] or (s.get('chassis_scheduler') or {}).get('maintenance')
      or require_idle and s['chassis_active']):
   raise ValueError('special_console_unready')
  mp,od,stamp=n.special_poses();n.validate_motion_proof(stamp);intent()
  chain=state['chain']
  if chain is None:
   chain=StraightChain(mp,od,[dict(id=state['key'][2],pose=mp.tolist())]);state['chain']=chain
  old=chain.previous_sample
  if old is not None and old[0]==stamp:
   if not np.array_equal(mp,old[1]) or not np.array_equal(od,old[2]):raise ValueError('motion_pose_changed_during_proof')
  else:chain.sample(mp,od,stamp,n.now())
  return mp,od,stamp
 def close():
  handle=state['handle']
  if handle is None:return True
  try:return n.end_output(handle)
  finally:
   # A short-TTL timeout fences this identity. Do not send an ordinary zero
   # command through the now-closed identity, or touch a newer goal's output.
   if n.transport.current==handle:
    n.active=False;n.intent.publish(Bool(data=False))
 intent()
 if state['complete']:return 'arrived_stopped'
 if not state['attempted']:
  if not n.transport.native_sdk:raise ValueError('special_native_sdk_required')
  n.wait_controller_quiet(epoch);proof(require_idle=True);intent()
  state['handle']=n.transport.current
  n.status('t1_contact_forward');n.waypoints.show('T1交接前贴靠：车头正前方0.225 SDK，单次2秒')
  began=time.monotonic()
  # Set before send: an acknowledgement timeout can still mean accepted motion.
  state['attempted']=True
  try:
   n.active=True;n.intent.publish(Bool(data=True))
   next_send=began
   while time.monotonic()-began<DURATION:
    proof()
    now=time.monotonic();remaining=DURATION-(now-began)
    if remaining<=0:break
    if now>=next_send:
     try:n.transport.send((SLOW_SDK,0.,0.),handle=state['handle'],ttl_s=min(COMMAND_TTL,remaining))
     except ValueError as exc:
      # Any uncertain acknowledgement ends this single attempt; never restart.
      if str(exc) in ('ROS chassis scheduler unavailable','ROS chassis management timeout','ROS chassis command acknowledgement timeout'):
       raise TimeoutError('t1_contact_output_uncertain '+str(exc)) from exc
      raise
     next_send=now+REFRESH_INTERVAL
    remaining=DURATION-(time.monotonic()-began)
    if remaining>0:n.spin(min(.02,remaining))
  finally:close()
 if not rclpy.ok():raise ValueError('ros_shutdown')
 if close() is False:raise ValueError('previous_output_end_unconfirmed')
 # The bounded pulse is over. Do not add a second measured-stop window before
 # handing off; retain ownership, cancellation and current-source validation.
 _,_,stamp=proof(require_idle=True)
 intent();n.validate_motion_proof(stamp)
 state['complete']=True
 n.status('t1_contact_handoff');n.waypoints.show('T1贴靠输出已结束，直接交接任务')
 return 'arrived_stopped'
