import unittest
from mission_route import MissionRoute,path_covers_vias,point

def p(kind,x=0.,ident=None):
 v={'type':kind,'pose':[x,0.,0.]}
 if ident:v['point_id']=ident
 return v

class Routes(unittest.TestCase):
 def test_groups_vias_and_waits_at_exact_task(self):
  r=MissionRoute('room');r.add(p('via',.2));r.add(p('task',.4,'point_1'));r.add(p('stop',.8))
  with self.assertRaisesRegex(ValueError,'worker'):r.start(set())
  goal=r.start({'point_1'});self.assertEqual(goal['type'],'via');self.assertEqual(goal['pose'][0],.2)
  self.assertEqual(len(r.vias()),1)
  r.observe([0,0,0],[0,0,0],1.);r.observe([.08,0,0],[.08,0,0],1.2);r.observe([.16,0,0],[.16,0,0],1.4)
  self.assertEqual(r.vias(),[])
  self.assertEqual(r.finish('arrived_stopped',pose=[.2,0,0])['point_id'],'point_1');self.assertEqual(r.mode,'task_direct')
  self.assertIsNone(r.finish('arrived_stopped'));self.assertEqual(r.state,'waiting_task');self.assertEqual(r.index,1)
  self.assertEqual(r.release_task()['pose'][0],.8)
  self.assertIsNone(r.finish('arrived_stopped'));self.assertEqual(r.state,'completed')
 def test_missed_via_cannot_report_terminal_success(self):
  r=MissionRoute('room');r.add(p('via',.4));r.add(p('task',1.,'point_2'));r.start({'point_2'})
  with self.assertRaisesRegex(ValueError,'unvisited'):r.finish('arrived_stopped')
 def test_tf_jump_and_backward_stamp_do_not_consume_via(self):
  r=MissionRoute('r');r.add(p('via',1));r.add(p('stop',2));r.start(set());r.observe([0,0,0],[0,0,0],1)
  with self.assertRaisesRegex(ValueError,'jump'):r.observe([1,0,0],[0,0,0],1.05)
  self.assertEqual(r.index,0);self.assertFalse(r.running)
  r.observe([1,0,0],[0,0,0],.9);self.assertEqual(r.index,0)
 def test_verified_stationary_gap_does_not_skip_or_disable_pending_via(self):
  r=MissionRoute('r');r.add(p('via',.3));r.add(p('stop',1));r.start(set());r.observe([0,0,0],[0,0,0],1.)
  self.assertFalse(r.observe([0,0,0],[0,0,0],2.));self.assertTrue(r.running);self.assertEqual(r.index,0)
  for i in range(1,6):r.observe([i*.05,0,0],[i*.05,0,0],2.+i*.12)
  self.assertEqual(r.index,1)
 def test_map_correction_after_gap_cannot_be_rebaselined_into_a_visit(self):
  r=MissionRoute('r');r.add(p('via',.15));r.add(p('stop',1));r.start(set());r.observe([0,0,0],[0,0,0],1.)
  with self.assertRaisesRegex(ValueError,'jump'):r.observe([.15,0,0],[0,0,0],1.5)
  r.observe([.15,0,0],[0,0,0],1.6);self.assertEqual(r.index,0)
 def test_yaw_frame_jump_cannot_mark_a_via(self):
  r=MissionRoute('r');r.add(p('via',.4));r.add(p('stop',1));r.start(set())
  r.observe([.26,0,0],[.26,0,0],1.)
  with self.assertRaisesRegex(ValueError,'jump'):r.observe([.31,0,1.],[.31,0,0.],1.1)
  self.assertEqual(r.index,0)
 def test_definition_load_is_draft_and_point_ids_are_not_actions(self):
  r=MissionRoute('r');r.add(p('task',1,'point_2'));v=r.definition();s=MissionRoute('r');s.load(v)
  self.assertFalse(s.running);self.assertEqual(s.state,'draft');self.assertEqual(s.points[0]['point_id'],'point_2')
  s.start({'point_2'})
  with self.assertRaisesRegex(ValueError,'running'):s.load(v)
 def test_repeated_start_does_not_change_mission_and_cancel_cannot_release(self):
  r=MissionRoute('r');r.add(p('task',0,'point_1'));r.start({'point_1'});m=r.mission_id
  self.assertIsNone(r.start({'point_1'}));self.assertEqual(r.mission_id,m)
  r.finish('arrived_stopped');r.cancel();self.assertIsNone(r.release_task())
 def test_two_visits_and_stop_compatibility(self):
  r=MissionRoute('r');r.add(p('task',1,'point_1'));r.add(p('task',2,'point_1'));r.start({'point_1'})
  r.finish('arrived_stopped');self.assertEqual(r.release_task()['pose'][0],2)
  r.finish('stopped: fault');self.assertFalse(r.running)
 def test_route_constraints_reject_missing_or_reversed_via(self):
  via=[point(p('via',.3)),point(p('via',.7))]
  self.assertTrue(path_covers_vias([[0,0,0],[1,0,0]],via))
  self.assertFalse(path_covers_vias([[0,0,0],[0,1,0],[1,1,0]],via))
  self.assertFalse(path_covers_vias([[1,0,0],[0,0,0]],via))
 def test_invalid_schema_and_infinite_positions(self):
  for value in ({'type':'task','pose':[0,0,0],'point_id':'place'},p('other'),p('via',float('nan'))):
   with self.assertRaises(ValueError):point(value)
if __name__=='__main__':unittest.main()
