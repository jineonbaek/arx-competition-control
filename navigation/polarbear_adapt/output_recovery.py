"""Recover a motion lease without changing the semantic goal or replaying output."""
import time
import rclpy
from std_msgs.msg import Bool
from recovery_budget import paused_budget
from chassis_command import output_unavailable

@paused_budget
def recover_output(n,epoch):
 from special_runtime import wait_special_ready,transient_input
 state=getattr(n,'output_recovery',None)
 if state is None or state['epoch']!=epoch:
  state=dict(epoch=epoch,handle=n.transport.current);n.output_recovery=state
 old=state['handle']
 n.status('recovering_chassis_output');n.waypoints.show('底盘指令超时：保留原导航目标，自动确认停车并恢复输出')
 while rclpy.ok():
  n.same_intent(epoch)
  if n.ready_file is not None and not n.ready_file.is_file():raise ValueError('execution_gate_closed')
  # A zero-velocity move may be rejected by an expired generation. Its scoped
  # end remains valid and must be acknowledged before allocating any new one.
  ended=n.end_output(old)
  if n.transport.current==old:
   n.active=False;n.intent.publish(Bool(data=False)) if hasattr(n,'intent') else None
  if ended is False:n.spin(.1);continue
  try:
   n.current_intent(epoch);n.wait_controller_quiet(epoch)
   contact=getattr(n,'t1_contact_session',None)
   if contact is not None and contact['epoch']==epoch:chain=contact['chain']
   else:
    session=getattr(n,'special_session',None)
    chain=session['chain'] if n._special_execution and session and session['epoch']==epoch else None
   wait_special_ready(n,epoch,chain)
   n.current_intent(epoch)
   # Contact is a once-only bounded attempt. After an uncertain attempt only
   # stopped verification follows; allocating a new motion identity is needless.
   attempted=contact is not None and contact['epoch']==epoch and contact['attempted']
   if not attempted:n.transport.renew_task(old)
   n.output_recovery=None;n.pending_fault=None
   if not n._special_execution and not (contact is not None and contact['epoch']==epoch):
    n.hold_for_replan('chassis_output_timeout')
   n.status('chassis_output_restored');return
  except Exception as exc:
   if not (output_unavailable(exc) or transient_input(exc) or isinstance(exc,ConnectionError)):raise
   n.spin(.1)
 raise ValueError('ros_shutdown')
