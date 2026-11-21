"""Scoped real T2 followed by navigation only; no ROS/transport."""
import unittest
from mission_route import MissionRoute
from t2_scope import t2_pair_indices

def task(ident,tag,x):return dict(id=ident,type='task',point_id=tag,pose=[x,0.,0.])
def nav(ident,kind,x):return dict(id=ident,type=kind,pose=[x,0.,0.])
class TailRoute(unittest.TestCase):
 def route(self,suffix):
  r=MissionRoute('map')
  for p in [task('old-t1','point_1',-3.),nav('entry','special',0.),task('t2','point_2',1.)]+suffix:r.add(p)
  return r
 def arrive(self,r):
  r.start_t2(['point_2'],[-2.,0.,0.]);self.assertEqual(r.index,1)
  self.assertEqual(r.finish('arrived_stopped',[0.,0.,0.])['id'],'t2')
  self.assertIsNone(r.finish('arrived_stopped',[1.,0.,0.]));self.assertEqual(r.state,'waiting_task')
 def test_real_t2_release_continues_each_navigation_type_and_ends_only_at_suffix(self):
  for kind in ['special','via','stop']:
   with self.subTest(kind=kind):
    r=self.route([nav('after',kind,2.)]);before=r.definition();self.arrive(r)
    p=r.release_task([1.02,0.,0.]);self.assertIsNotNone(p,'T2 release ended before suffix');self.assertEqual(p['id'],'after')
    self.assertTrue(r.running);self.assertEqual(r.required_task_ids(),set());self.assertEqual(r.definition(),before)
    self.assertIsNone(r.finish('arrived_stopped',[2.,0.,0.]));self.assertFalse(r.running);self.assertEqual(r.state,'completed')
 def test_t2_index_is_independent_of_navigation_end_and_entry_task_only_navigates(self):
  r=MissionRoute('map')
  for p in [task('entry-t1','point_1',0.),task('t2','point_2',1.),nav('after','special',2.),nav('last','special',3.)]:r.add(p)
  before=r.definition();r.start_t2(['point_2'],[-2.,0.,0.])
  self.assertEqual(getattr(r,'t2_index',None),1);self.assertEqual(r.execution_end,4);self.assertEqual(r.required_task_ids(),{'point_2'})
  self.assertEqual(r.finish('arrived_stopped',[0.,0.,0.])['id'],'t2');r.finish('arrived_stopped',[1.,0.,0.]);self.assertEqual(r.state,'waiting_task')
  p=r.release_task([1.05,0.,0.]);self.assertEqual(p['id'],'last');self.assertEqual(r.mode,'direct');self.assertEqual(r.direct_source['pose'],[1.05,0.,0.]);self.assertEqual(r.definition(),before)
 def test_later_task_or_test_marker_is_rejected_before_start(self):
  for suffix in [[task('later-t1','point_1',3.)],[dict(nav('test','special',3.),test_task_id='point_1')]]:
   with self.subTest(suffix=suffix):
    r=self.route(suffix);before=r.definition()
    with self.assertRaises(ValueError):t2_pair_indices(r.points)
    with self.assertRaises(ValueError):r.start_t2(['point_2'],[-2.,0.,0.])
    self.assertFalse(r.running);self.assertEqual(r.definition(),before)
 def test_no_suffix_still_ends_after_t2(self):
  r=self.route([]);self.arrive(r);self.assertIsNone(r.release_task([1.,0.,0.]));self.assertEqual(r.state,'completed')
 def test_cancel_during_suffix_keeps_all_points_and_cannot_advance(self):
  r=self.route([nav('after','special',2.)]);before=r.definition();self.arrive(r)
  self.assertIsNotNone(r.release_task([1.,0.,0.]));r.cancel();self.assertIsNone(r.finish('arrived_stopped',[2.,0.,0.]));self.assertEqual(r.definition(),before);self.assertEqual(r.state,'cancelled')
if __name__=='__main__':unittest.main()
