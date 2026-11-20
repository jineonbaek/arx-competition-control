import unittest
from chassis_pose_policy import PosePolicy
class PoseTests(unittest.TestCase):
 def setUp(self):self.p=PosePolicy()
 def feed(self,t,alignment=(0,0,0),now=None):
  return self.p.accept(int(t*1e9),int((t+.31 if now is None else now)*1e9),alignment)
 def test_three_different_stamps_then_stays_valid_at_measured_delay(self):
  self.assertFalse(self.feed(10));self.assertFalse(self.feed(10))
  self.assertFalse(self.feed(10.1));self.assertTrue(self.feed(10.2))
  self.assertTrue(self.feed(10.3,now=10.634))
 def test_invalid_then_three_new_samples(self):
  for t in (10,10.1,10.2):self.feed(t)
  self.assertFalse(self.feed(10.3,now=11))
  self.assertFalse(self.feed(11.1));self.assertFalse(self.feed(11.2));self.assertTrue(self.feed(11.3))
 def test_alignment_jump_withdraws_and_requires_new_samples(self):
  for t in (10,10.1,10.2):self.feed(t)
  self.assertFalse(self.feed(10.3,(.3,0,0)))
  self.assertFalse(self.feed(10.3,(.3,0,0)))
  self.assertFalse(self.feed(10.4,(.3,0,0)))
  self.assertTrue(self.feed(10.5,(.3,0,0)))
 def test_clock_reversal_and_explicit_relocalization(self):
  for t in (10,10.1,10.2):self.feed(t)
  self.assertFalse(self.feed(9))
  self.p.reset()
  self.assertFalse(self.feed(10.3));self.assertFalse(self.feed(10.4));self.assertTrue(self.feed(10.5))
if __name__=='__main__':unittest.main()

class LeaseTests(unittest.TestCase):
 def test_expired_or_rolled_back_pose_cannot_revive(self):
  from chassis_pose_policy import SnapshotLease
  c=SnapshotLease();self.assertTrue(c.offer(10_000_000_000,10_400_000_000))
  self.assertTrue(c.usable(10_400_000_000));self.assertFalse(c.usable(10_700_000_000))
  self.assertFalse(c.usable(10_400_000_000));self.assertFalse(c.offer(10_000_000_000,10_400_000_000))
  self.assertTrue(c.offer(10_100_000_000,10_400_000_000))
 def test_reset_rejects_delayed_pre_reset_snapshot(self):
  from chassis_pose_policy import SnapshotLease
  c=SnapshotLease();c.offer(10_000_000_000,10_200_000_000);c.withdraw(10_200_000_000)
  self.assertFalse(c.offer(10_100_000_000,10_300_000_000))
  self.assertTrue(c.offer(10_300_000_000,10_400_000_000))
