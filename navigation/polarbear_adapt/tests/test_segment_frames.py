import unittest
from segment_frames import SegmentFrames

class SegmentTests(unittest.TestCase):
 def pair(self,q,stamp=1_000_000_000,frame='level'):
  q.add(stamp,frame,'ground','g');q.add(stamp,frame,'obstacle','o')
 def test_complete_pair_waits_and_retries_without_restamping(self):
  q=SegmentFrames();self.pair(q);out=[]
  q.drain(1_100_000_000,lambda *a:None,lambda *x:out.append(x));self.assertEqual(out,[])
  q.drain(1_200_000_000,lambda *a:(1.,2.,.23),lambda *x:out.append(x))
  self.assertEqual(out,[({'ground':'g','obstacle':'o'},(1.,2.,.23))]);self.assertEqual(q.completed,1_000_000_000)
 def test_source_metadata_wins_over_missing_or_different_tf(self):
  q=SegmentFrames();q.remember(1_000_000_000,'level',(1.,2.,.23));self.pair(q);out=[]
  q.drain(1_100_000_000,lambda *a:(_ for _ in ()).throw(AssertionError('must use matching metadata')),lambda *x:out.append(x))
  self.assertEqual(out[0][1],(1.,2.,.23))
 def test_wrong_frame_metadata_is_not_reused(self):
  q=SegmentFrames();q.remember(1_000_000_000,'other',(9.,9.,9.));self.pair(q);out=[]
  q.drain(1_100_000_000,lambda *a:None,lambda *x:out.append(x));self.assertEqual(out,[])
 def test_mismatched_pair_frames_cannot_form_a_cloud(self):
  q=SegmentFrames();q.add(1_000_000_000,'a','ground','g');q.add(1_000_000_000,'b','obstacle','o');out=[]
  q.drain(1_100_000_000,lambda *a:(0.,0.,0.),lambda *x:out.append(x));self.assertEqual(out,[])
 def test_expired_and_future_pairs_never_publish(self):
  for now in (1_580_000_001,800_000_000):
   q=SegmentFrames();self.pair(q);out=[];q.drain(now,lambda *a:(0.,0.,0.),lambda *x:out.append(x));self.assertEqual(out,[])
 def test_duplicates_and_late_packets_do_not_publish_twice(self):
  q=SegmentFrames();self.pair(q);out=[];publish=lambda *x:out.append(x)
  q.drain(1_100_000_000,lambda *a:(0.,0.,0.),publish);self.pair(q)
  q.drain(1_200_000_000,lambda *a:(0.,0.,0.),publish);self.assertEqual(len(out),1)
 def test_partial_queue_is_bounded_and_cannot_block_new_complete_frame(self):
  q=SegmentFrames(limit=3)
  for stamp in range(1_000_000_000,1_050_000_000,10_000_000):q.add(stamp,'level','ground','partial')
  self.assertLessEqual(len(q.pairs),3);self.pair(q,1_100_000_000);out=[]
  q.drain(1_200_000_000,lambda *a:(0.,0.,0.),lambda *x:out.append(x));self.assertEqual(len(out),1)
  self.assertEqual(len(q.pairs),0)
 def test_complete_frames_wait_in_order_for_matching_geometry(self):
  q=SegmentFrames();self.pair(q);self.pair(q,1_100_000_000);q.remember(1_100_000_000,'level',(2.,0.,0.));out=[]
  q.drain(1_200_000_000,lambda *a:None,lambda *x:out.append(x));self.assertEqual(out,[])
  q.remember(1_000_000_000,'level',(1.,0.,0.));q.drain(1_200_000_000,lambda *a:None,lambda *x:out.append(x))
  self.assertEqual([x[1][0] for x in out],[1.,2.])
 def test_nonfinite_origin_is_not_used(self):
  q=SegmentFrames();q.remember(1_000_000_000,'level',(float('nan'),0.,0.));self.pair(q);out=[]
  q.drain(1_100_000_000,lambda *a:(float('nan'),0.,0.),lambda *x:out.append(x));self.assertEqual(out,[])
if __name__=='__main__':unittest.main()
