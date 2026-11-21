import unittest,numpy as np
try:from agent.t2_recovery_path import reverse_issued_path
except ModuleNotFoundError:reverse_issued_path=None
class ReversePath(unittest.TestCase):
 def setUp(self):self.assertIsNotNone(reverse_issued_path,'recorded recovery path helper missing')
 def test_interrupted_path_returns_only_executed_prefix(self):
  p=np.zeros((3,6));p[:,0]=[0.,.5,1.];cur=np.zeros(6);cur[0]=.3
  out=reverse_issued_path([dict(points=p.tolist(),complete=False)],cur)
  np.testing.assert_allclose(out[0],cur);np.testing.assert_allclose(out[-1],p[0]);self.assertLessEqual(out[:,0].max(),.3+1e-9)
 def test_changed_measured_arm_not_on_issued_path_rejected(self):
  p=np.zeros((2,6));p[-1,0]=1.;cur=np.full(6,.5)
  with self.assertRaises(ValueError):reverse_issued_path([dict(points=p.tolist(),complete=False)],cur)
 def test_completed_history_is_reversed_without_new_waypoints(self):
  a=np.zeros((2,6));a[-1,0]=.2;b=np.array([a[-1],a[-1]*2]);out=reverse_issued_path([dict(points=a.tolist(),complete=True),dict(points=b.tolist(),complete=True)],b[-1]);np.testing.assert_allclose(out[-1],a[0]);self.assertLessEqual(out[:,0].max(),.4)
 def test_partial_normal_return_does_not_reenter_cabinet(self):
  def pts(xs):
   q=np.zeros((len(xs),6));q[:,0]=xs;return q.tolist()
  cur=np.zeros(6);cur[0]=.7
  out=reverse_issued_path([dict(points=pts([0,.5,1]),complete=True),dict(points=pts([1,.85,.7]),complete=False)],cur)
  self.assertLessEqual(out[:,0].max(),.7+1e-9);self.assertEqual(out[-1,0],0.)
 def test_recovery_stops_at_recorded_preobservation_goal(self):
  q=np.zeros((4,6));q[:,0]=[0,.2,.5,1.];goal=q[1];cur=q[-1]
  out=reverse_issued_path([dict(points=q.tolist(),complete=True)],cur,goal=goal)
  np.testing.assert_allclose(out[-1],goal);self.assertGreaterEqual(out[:,0].min(),.2-1e-9)
 def test_withdrawal_tracking_offset_does_not_reenter(self):
  q=np.zeros((4,6));q[:,0]=[0,.5,1,.7];q[-1,1]=.001
  out=reverse_issued_path([dict(points=q.tolist(),complete=False)],q[-1],goal=q[0])
  self.assertLessEqual(out[:,0].max(),.7+1e-9)
 def test_unreached_goal_in_first_segment_is_not_return_endpoint(self):
  q=np.zeros((2,6));q[-1,0]=1.;cur=q[0].copy();cur[0]=.3;goal=q[0].copy();goal[0]=.8
  out=reverse_issued_path([dict(points=q.tolist(),complete=False)],cur,goal=goal)
  self.assertLessEqual(out[:,0].max(),.3+1e-9);np.testing.assert_allclose(out[-1],q[0])
class MeasuredHistory(unittest.TestCase):
 def test_interrupted_motion_retains_feedback_not_unissued_loaded_references(self):
  from agent.t2_recovery_path import RecordingMotion
  q=np.zeros(6)
  class Motion:
   def arm(self,segments,guard):
    q[0]=.1;guard();q[0]=.2;guard();raise ValueError('cancelled')
  recording=RecordingMotion(Motion(),lambda:q.tolist())
  with self.assertRaises(ValueError):recording.arm([dict(joint_waypoints_rad=[[0.]*6,[1.]*6])],lambda:True)
  h=recording.snapshot_paths();self.assertFalse(h[0]['complete']);self.assertLessEqual(np.asarray(h[0]['points'])[:,0].max(),.2)
  out=reverse_issued_path(h,q);self.assertLessEqual(out[:,0].max(),.2);np.testing.assert_allclose(out[-1],np.zeros(6))
if __name__=='__main__':unittest.main()
