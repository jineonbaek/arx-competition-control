"""Existing measured base correction with inert controller and virtual pose gaps."""
import threading,unittest
from types import SimpleNamespace as NS
import numpy as np
from web.src.docking_motion import DockingMotion

class DockingPoseWait(unittest.TestCase):
 def test_pose_gap_does_not_rebase_goal_or_consume_the_motion_deadline(self):
  clock=[10.];pose=np.zeros(3);moves=[];command=np.zeros(3);sent=[False]
  def submit(m,token):
   if m['type']=='chassis.end_output':command[:]=0.
   else:command[:]=[m['x'],m['y'],m['omega']];moves.append(dict(m));sent[0]=True
   return {'ok':True}
  def snapshot():return {'arms':{'right':{'joints':[0.]*6}},'body':{'height':0.}}
  def sleep(dt):pose[:]+=command*dt*10.;clock[0]+=dt
  class Reader:
   def now(self):return clock[0]
   def read(self):
    if sent[0] and clock[0]<14.:raise ValueError('motion_pose_stale_or_invalid')
    return pose.copy(),pose.copy(),clock[0]
   def check_motion(self,*a):pass
   def close(self):pass
  ctrl=NS(clock=lambda:clock[0],lock=threading.RLock(),submit=submit,snapshot=snapshot,motion_valid=lambda t:True)
  d=DockingMotion(ctrl,'task',guard=lambda *a:True,sleep=sleep,base_factory=lambda v:Reader(),base_client='task',timeout_s=2.5)
  result=d.base([-.006,0,0],lambda:None)
  self.assertIsInstance(result,dict);self.assertGreater(clock[0],14.);self.assertGreaterEqual(len(moves),2);self.assertTrue(all(m['x']<=0 for m in moves));self.assertGreater(pose[0],-.014)
if __name__=='__main__':unittest.main()
