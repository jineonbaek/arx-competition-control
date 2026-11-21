import unittest,math
from geometry_msgs.msg import PoseArray,Pose
from t2_owned_return import ReturnControl
class OwnedReturn(unittest.TestCase):
 def message(self,t,x=0.):
  m=PoseArray();m.header.frame_id='map';m.header.stamp.sec=int(t);m.header.stamp.nanosec=round((t-int(t))*1e9)
  for _ in range(2):p=Pose();p.position.x=x;p.orientation.w=1.;m.poses.append(p)
  return m
 def test_stale_or_withdrawn_authority_outputs_zero(self):
  c=ReturnControl([.4,0.,0.]);self.assertEqual(c.update(self.message(100.),102.)['command'],[0.,0.,0.]);self.assertEqual(c.update(PoseArray(),102.)['status'],'waiting')
 def test_fresh_pair_produces_only_slow_sdk_candidates(self):
  c=ReturnControl([.4,0.,0.]);c.update(self.message(100.),100.);r=c.update(self.message(100.1),100.1);self.assertGreater(r['command'][0],0.);self.assertLessEqual(math.hypot(*r['command'][:2]),.225001)
 def test_at_goal_requires_new_frames_and_stopped_proof(self):
  c=ReturnControl([0.,0.,0.]);r=None
  for i in range(20):
   t=100.+i*.1;r=c.update(self.message(t),t)
   if r.get('status')=='completed':break
  self.assertEqual(r['status'],'completed');self.assertTrue(r['stationary']);self.assertEqual(r['command'],[0.,0.,0.])
if __name__=='__main__':unittest.main()
