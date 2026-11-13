"""Read-only audit of raw SI, guarded SI and estimated SDK topics."""
import json,math,sys,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
rclpy.init();n=Node('mppi_readonly_shadow_audit');rows={k:[] for k in ('raw','guarded','sdk','state')}
for k,t in [('raw','raw_velocity'),('guarded','preview_velocity'),('sdk','sdk_preview')]:
 n.create_subscription(TwistStamped,'/arx_mppi_trial/'+t,lambda m,k=k:rows[k].append([time.monotonic(),m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]),100)
n.create_subscription(String,'/arx_mppi_trial/preview_status',lambda m:rows['state'].append([time.monotonic(),m.data]),100)
end=time.monotonic()+float(sys.argv[2])
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
summary={'physical_commands':0}
for k in ('raw','guarded','sdk'):
 a=rows[k];summary[k]={'count':len(a),'backward_peak':max([-x[1] for x in a],default=0),'last':a[-1][1:] if a else None}
 if a:assert all(math.isfinite(v) for x in a for v in x[1:])
a=rows['sdk'];summary['sdk_over_budget']=sum(math.hypot(x,y)/.22+abs(w)/.055>1.000001 for _,x,y,w in a)
summary['sdk_at_known_backward_response']=sum(x<=-.14 for _,x,y,w in a)
summary['states']=sorted(set(x[1] for x in rows['state']))
Path(sys.argv[1]).write_text(json.dumps({'summary':summary,'trace':rows},indent=2));print(json.dumps(summary,indent=2))
assert summary['sdk_over_budget']==0 and a and a[-1][1:]==[0.,0.,0.]
n.destroy_node();rclpy.shutdown()
