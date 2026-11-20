"""Runs ONLY on isolated domain 199 with a fake CAN driver."""
import json,threading,time,unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import SetParametersResult
from arx_chassis_interfaces.msg import Output
from arx_chassis.node import SchedulerNode
from arx_chassis.client import Transport,Session
class RosIntegration(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  rclpy.init(domain_id=199)
  cls.node=SchedulerNode(dict(execution_enabled=True,body=dict(chassis_verified=True,chassis_stop_verified=True,max_linear_m_s=1.6,max_angular_rad_s=1.8,manual_sdk_axes={'x':.92,'y':1.6})))
  cls.fake=rclpy.create_node('isolated_fake_driver');cls.bound='';cls.outputs=[];cls.seq=0;cls.sdk=[0.,0.,0.]
  cls.pub=cls.fake.create_publisher(String,'/web/chassis/output_status',1)
  def bind(req,res):
   cls.bound=req.parameters[0].value.string_value;res.results=[SetParametersResult(successful=True)];return res
  cls.fake.create_service(SetParameters,'/lift_controller/set_parameters',bind)
  def output(m):
   cls.outputs.append(m);cls.seq=m.output_seq;cls.sdk=list(m.sdk)
  cls.output_sub=cls.fake.create_subscription(Output,'/chassis/ordered_output',output,1)
  def status():
   m=String();m.data=json.dumps(dict(driver_instance='FAKE-NO-HARDWARE',ordered_output=True,calibrated=True,applied_mode=2,applied_sdk=cls.sdk,output_seq=cls.seq,scheduler_instance=cls.bound));cls.pub.publish(m)
  cls.fake.create_timer(.02,status)
  cls.exec=SingleThreadedExecutor();cls.exec.add_node(cls.node);cls.exec.add_node(cls.fake)
  cls.thread=threading.Thread(target=cls.exec.spin,daemon=True);cls.thread.start()
  cls.transport=Transport(domain=199)
  deadline=time.monotonic()+8
  while not (cls.transport.state and cls.transport.state.ready) and time.monotonic()<deadline:time.sleep(.05)
  assert cls.transport.state and cls.transport.state.ready
 @classmethod
 def tearDownClass(cls):
  cls.transport.close();cls.exec.shutdown(timeout_sec=2);cls.thread.join(2);cls.node.destroy_node();cls.fake.destroy_node();rclpy.shutdown()
 def payload(self,source,seq=1,ttl=.3,kind='chassis.nav_sdk'):
  return dict(type=kind,source_id=source,task_id='test',generation=1,seq=seq,issued_at=time.time(),ttl_s=ttl,x=.1,y=0.,omega=0.)
 def test_01_typed_command_expiry_and_result(self):
  a=Session('navigation',self.transport);res=a.send(self.payload('navigation-test'))
  self.assertTrue(res['accepted']);time.sleep(.45)
  self.assertFalse(self.transport.state.active);self.assertEqual(self.sdk,[0.,0.,0.])
 def test_01b_navigation_one_second_lease_is_accepted_on_the_wire(self):
  a=Session('navigation',self.transport);a.send(self.payload('navigation-one-second',ttl=1.))
  time.sleep(.4);self.assertTrue(self.transport.state.active);a.manage('end_client')
 def test_02_other_client_cleanup_is_scoped(self):
  a=Session('manual',self.transport);b=Session('navigation',self.transport)
  a.send(self.payload('web-test',ttl=1,kind='chassis'));b.send(self.payload('navigation-b'))
  a.manage('end_client');self.assertEqual(self.transport.state.client_id,b.client_id);b.manage('end_client')
 def test_03_old_scheduler_instance_rejected(self):
  a=Session('navigation',self.transport);a.register()
  with self.assertRaises(ValueError):self.transport.send(self.payload('old'),a.client_id,'old')
 def test_04_maintenance_exclusive(self):
  a=Session('manual',self.transport);a.manage('reserve');b=Session('navigation',self.transport)
  with self.assertRaises(ValueError):b.send(self.payload('blocked'))
  a.manage('release')
 def test_05_unverified_chassis_rejected(self):
  self.node.config['body']['chassis_verified']=False
  try:
   a=Session('navigation',self.transport)
   with self.assertRaises(ValueError):a.send(self.payload('unverified'))
  finally:self.node.config['body']['chassis_verified']=True
 def test_06_missing_output_reader_not_ready(self):
  self.fake.destroy_subscription(self.output_sub)
  deadline=time.monotonic()+2.
  while self.transport.state.ready and time.monotonic()<deadline:time.sleep(.02)
  self.assertFalse(self.transport.state.ready)
  a=Session('navigation',self.transport)
  with self.assertRaises(ValueError):a.send(self.payload('no-reader'))
