"""Await lifecycle services and transition completion without duplicate requests."""
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState,GetState
rclpy.init();n=Node('arx_mppi_lifecycle_wait')
change=n.create_client(ChangeState,'/arx_mppi_trial/controller_server/change_state');get=n.create_client(GetState,'/arx_mppi_trial/controller_server/get_state')
def await_future(f,seconds=40):
 end=time.monotonic()+seconds
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 if not f.done():raise RuntimeError('lifecycle response timeout')
 return f.result()
end=time.monotonic()+25
while time.monotonic()<end:
 if change.service_is_ready() and get.service_is_ready():break
 rclpy.spin_once(n,timeout_sec=.1)
else:raise RuntimeError('lifecycle services unavailable')
for transition,target,expected in [(1,2,1),(3,3,2)]:
 state=await_future(get.call_async(GetState.Request())).current_state.id
 if state==target:continue
 if state!=expected:raise RuntimeError(f'unexpected lifecycle state {state}')
 request=ChangeState.Request();request.transition.id=transition
 if not await_future(change.call_async(request)).success:raise RuntimeError('transition rejected')
 if await_future(get.call_async(GetState.Request())).current_state.id!=target:raise RuntimeError('target state not reached')
print('controller lifecycle active',flush=True);n.destroy_node();rclpy.shutdown()
