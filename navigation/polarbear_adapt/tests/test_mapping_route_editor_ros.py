import os,json,unittest
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import rclpy
from std_msgs.msg import String
from mapping_route_editor import MappingRouteEditor
class Editor(unittest.TestCase):
 @classmethod
 def setUpClass(cls):rclpy.init()
 @classmethod
 def tearDownClass(cls):rclpy.shutdown()
 def setUp(self):self.n=MappingRouteEditor()
 def tearDown(self):self.n.destroy_node()
 def test_route_editor_has_no_chassis_transport_and_cannot_start(self):
  w=self.n.waypoints;w.add_json(String(data=json.dumps(dict(type='task',point_id='point_1',pose=[1,0,0]))));w.start(None)
  self.assertFalse(hasattr(self.n,'transport'));self.assertFalse(w.route.running);self.assertIsNone(self.n.target)
 def test_tasks_added_while_testing_never_become_real_jobs(self):
  from geometry_msgs.msg import PoseArray,Pose
  w=self.n.waypoints;w.add_json(String(data=json.dumps(dict(type='task',point_id='point_1',pose=[1,0,0]))));w.edit(String(data=json.dumps(dict(action='test_mode',enabled=True))))
  w.add_json(String(data=json.dumps(dict(type='task',point_id='point_2',pose=[2,0,0]))))
  m=PoseArray();m.header.frame_id='map';m.header.stamp=self.n.get_clock().now().to_msg();p=Pose();p.orientation.w=1.;m.poses=[p,p];self.n.snapshot=m
  w.add_current(String(data=json.dumps(dict(request_id='new-current',kind='point_1'))))
  self.assertEqual(len(w.route.points),3);self.assertTrue(all(p['type']=='special' and p.get('test_task_id') for p in w.route.points))
 def test_mode_roundtrip_and_transactional_reload(self):
  w=self.n.waypoints;w.add_json(String(data=json.dumps(dict(type='task',point_id='point_2',pose=[1,0,0]))));normal=w.route.definition();w.edit(String(data=json.dumps(dict(action='test_mode',enabled=True))));self.assertEqual(w.route.points[0]['test_task_id'],'point_2')
  w.edit(String(data=json.dumps(dict(action='test_mode',enabled=False))));self.assertEqual(w.route.definition(),normal)
  expected=w.route.definition();w.add_json(String(data=json.dumps(dict(type='special',pose=[2,0,0]))));edited=w.route.definition();w.edit(String(data=json.dumps(dict(action='replace_if_unchanged',definition=normal,expected_definition=expected))));self.assertEqual(w.route.definition(),edited)
  w.edit(String(data=json.dumps(dict(action='replace_if_unchanged',definition=normal,expected_definition=edited))));self.assertEqual(w.route.definition(),normal)
if __name__=='__main__':unittest.main()
