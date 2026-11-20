import os,json,copy,unittest,sys
from pathlib import Path
sys.path.insert(0,os.getenv("ARX_EDIT_MODULE_DIR",str(Path(__file__).resolve().parents[1])))
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray,Pose
from mapping_route_editor import MappingRouteEditor
class CurrentPointEdit(unittest.TestCase):
 @classmethod
 def setUpClass(cls):rclpy.init()
 @classmethod
 def tearDownClass(cls):rclpy.shutdown()
 def setUp(self):
  self.n=MappingRouteEditor();self.w=self.n.waypoints
  for v in [dict(id='a',type='special',pose=[1,0,0]),dict(id='b',type='task',point_id='point_1',name='取物',pose=[2,0,0]),dict(id='c',type='task',point_id='point_2',pose=[3,0,0])]:self.w.add_json(String(data=json.dumps(v)))
 def tearDown(self):self.n.destroy_node()
 def edit(self,v):self.w.edit(String(data=json.dumps(v)))
 def snapshot(self):
  m=PoseArray();m.header.frame_id='map';m.header.stamp=self.n.get_clock().now().to_msg();p=Pose();p.position.x=4.;p.position.y=5.;p.orientation.z=0.29552020666133955;p.orientation.w=0.955336489125606;m.poses=[p,p];self.n.snapshot=m
 def request(self):
  return dict(action='replace_current',id='b',route_id=self.w.route.route_id,expected_point=copy.deepcopy(self.w.route.points[1]),request_id='capture-once')
 def test_replacement_preserves_point_identity_type_and_other_points(self):
  before=copy.deepcopy(self.w.route.points);self.snapshot();self.edit(self.request());p=self.w.route.points[1]
  self.assertAlmostEqual(p['pose'][0],4.);self.assertAlmostEqual(p['pose'][1],5.);self.assertAlmostEqual(p['pose'][2],.6)
  for key in ('id','type','point_id','name'):self.assertEqual(p[key],before[1][key])
  self.assertEqual(self.w.route.points[0],before[0]);self.assertEqual(self.w.route.points[2],before[2]);self.assertFalse(self.w.route.running);self.assertIsNone(self.n.target)
 def test_stale_point_missing_pose_or_active_target_never_replaces(self):
  before=copy.deepcopy(self.w.route.points);req=self.request();self.edit(req);self.assertEqual(self.w.route.points,before)
  self.snapshot();req['expected_point']['pose']=[99,0,0];self.edit(req);self.assertEqual(self.w.route.points,before)
  req=self.request();self.n.target=[0,0,0];self.edit(req);self.assertEqual(self.w.route.points,before)
 def test_repeated_move_requests_follow_same_id_to_both_ends(self):
  up=dict(action='move',id='c',delta=-1,route_id=self.w.route.route_id)
  for _ in range(5):self.edit(up)
  self.assertEqual([p['id'] for p in self.w.route.points],['c','a','b'])
  down=dict(up,delta=1)
  for _ in range(5):self.edit(down)
  self.assertEqual([p['id'] for p in self.w.route.points],['a','b','c'])
 def test_replacement_follows_selected_id_after_move_and_rejects_old_route(self):
  req=self.request();self.edit(dict(action='move',id='b',delta=-1,route_id=self.w.route.route_id));self.snapshot();self.edit(req)
  self.assertEqual(self.w.route.points[0]['id'],'b');self.assertAlmostEqual(self.w.route.points[0]['pose'][0],4.)
  old=copy.deepcopy(self.w.route.points);self.edit(dict(action='move',id='b',delta=1,route_id='old-route'));self.assertEqual(self.w.route.points,old)
if __name__=='__main__':unittest.main()
