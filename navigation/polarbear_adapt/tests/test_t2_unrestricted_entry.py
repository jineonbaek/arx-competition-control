"""T2 scope restores current behavior without changing saved point types."""
import unittest
from mission_route import MissionRoute
from t2_scope import t2_pair_indices

class UnrestrictedEntry(unittest.TestCase):
 def route(self,kind):
  r=MissionRoute('map')
  for p in [dict(id='earlier-t1',type='task',point_id='point_1',pose=[-5.,0.,0.]),dict(id='entry',type=kind,pose=[0.,0.,0.],**({'point_id':'point_1'} if kind=='task' else {})),dict(id='t2',type='task',point_id='point_2',pose=[1.,0.,0.]),dict(id='later',type='special',pose=[2.,0.,0.]),dict(id='later-stop',type='stop',pose=[3.,0.,0.])]:r.add(p)
  return r
 def test_predecessor_type_is_not_a_requirement(self):
  for kind in ['special','via','stop','task']:
   with self.subTest(kind=kind):self.assertEqual(t2_pair_indices(self.route(kind).points),(1,2))
 def test_entry_is_navigation_only_and_t2_ends_scope_without_replacing_definition(self):
  for kind in ['special','via','stop','task']:
   with self.subTest(kind=kind):
    r=self.route(kind);before=r.definition();first=r.start_t2(['point_2'],[-3.,0.,0.])
    self.assertEqual(first['id'],'entry');self.assertEqual(r.mode,'direct');self.assertEqual(r.direct_points()[0]['type'],kind)
    self.assertEqual(r.required_task_ids(),{'point_2'})
    target=r.finish('arrived_stopped',[0.,0.,0.]);self.assertEqual(target['id'],'t2');self.assertEqual(r.mode,'task_direct')
    self.assertIsNone(r.finish('arrived_stopped',[1.,0.,0.]));self.assertEqual(r.state,'waiting_task')
    self.assertEqual(r.release_task([1.,0.,0.])['id'],'later');self.assertTrue(r.running);self.assertEqual(r.required_task_ids(),set());self.assertEqual(r.finish('arrived_stopped',[2.,0.,0.])['id'],'later-stop');self.assertIsNone(r.finish('arrived_stopped',[3.,0.,0.]));self.assertFalse(r.running);self.assertEqual(r.definition(),before)
 def test_t1_predecessor_never_invokes_t1_wall_contact(self):
  from types import SimpleNamespace
  from t1_contact_runtime import point_key
  r=self.route('task');r.start_t2(['point_2'],[-3.,0.,0.])
  self.assertIsNone(point_key(SimpleNamespace(waypoints=SimpleNamespace(route=r))))
 def test_unique_formal_t2_and_predecessor_still_required(self):
  for points in [[],[dict(type='task',point_id='point_2')],[dict(type='special'),dict(type='task',point_id='point_2'),dict(type='task',point_id='point_2')],[dict(type='special',test_task_id='point_1'),dict(type='task',point_id='point_2')]]:
   with self.subTest(points=points),self.assertRaises(ValueError):t2_pair_indices(points)
 def test_normal_start_keeps_legacy_special_and_both_workers(self):
  r=self.route('special');before=r.definition()
  with self.assertRaises(ValueError):r.start(['point_2'],pose=[-5.,0.,0.])
  r.start(['point_1','point_2'],pose=[-5.,0.,0.]);self.assertIsNone(r.execution_end);self.assertEqual(r.required_task_ids(),{'point_1','point_2'});self.assertEqual(r.definition(),before)
if __name__=='__main__':unittest.main()
