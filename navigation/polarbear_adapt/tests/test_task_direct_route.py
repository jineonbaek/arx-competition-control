import unittest
from mission_route import MissionRoute

class TaskDirectRoute(unittest.TestCase):
 def route(self):
  r=MissionRoute('room')
  for v in (dict(type='via',pose=[.2,0,0]),dict(type='via',pose=[.4,0,0]),dict(type='task',point_id='point_1',pose=[.6,0,.3]),dict(type='via',pose=[.8,0,0]),dict(type='task',point_id='point_2',pose=[1.,0,.4])):r.add(v)
  return r
 def test_navigation_still_covers_vias_before_direct_task(self):
  r=self.route();goal=r.start({'point_1','point_2'},pose=[0,0,0])
  self.assertEqual(goal['pose'],[.4,0,0]);self.assertEqual(r.mode,'navigation')
  self.assertEqual([p['pose'][0] for p in r.vias()],[.2,.4])
  with self.assertRaisesRegex(ValueError,'unvisited'):r.finish('arrived_stopped',pose=[.4,0,0])
 def test_last_via_then_task_remains_task_and_waits_for_handoff(self):
  r=self.route();r.start({'point_1','point_2'},pose=[0,0,0])
  r.observe([.2,0,0],[.2,0,0],1.)
  r.observe([.31,0,0],[.31,0,0],1.3)
  r.observe([.4,0,0],[.4,0,0],1.6)
  p=r.finish('arrived_stopped',pose=[.4,0,0]);self.assertEqual(r.mode,'task_direct')
  self.assertEqual(p['point_id'],'point_1');self.assertEqual(p['pose'],[.6,0,.3]);self.assertEqual(r.vias(),[])
  self.assertEqual(r.direct_points(),[p]);self.assertIsNone(r.finish('arrived_stopped',pose=[.6,0,.3]))
  self.assertEqual(r.state,'waiting_task')
  p=r.release_task(pose=[.6,0,.3]);self.assertEqual(r.mode,'navigation');self.assertEqual(p['pose'][0],.8)
  r.observe([.8,0,0],[.8,0,0],2.)
  p=r.finish('arrived_stopped',pose=[.8,0,0]);self.assertEqual(r.mode,'task_direct');self.assertEqual(p['point_id'],'point_2')
 def test_task_at_start_uses_direct_mode_without_dropping_heading(self):
  r=MissionRoute('r');r.add(dict(type='task',point_id='point_1',pose=[.3,.2,-1.]))
  p=r.start({'point_1'},pose=[0,0,0]);self.assertEqual(r.mode,'task_direct');self.assertEqual(r.direct_points(),[p])
 def test_special_chain_finishes_before_next_oriented_task(self):
  r=MissionRoute('r')
  for x in (0.,.3):r.add(dict(type='special',pose=[x,0,0]))
  r.add(dict(type='task',point_id='point_1',pose=[.6,0,1.]))
  r.start({'point_1'},pose=[0,0,0]);self.assertEqual(r.mode,'direct')
  p=r.finish('arrived_stopped',pose=[.3,0,0]);self.assertEqual(p['point_id'],'point_1');self.assertEqual(r.mode,'task_direct')
 def test_two_cm_last_via_cannot_switch_to_direct_when_four_cm_away(self):
  r=MissionRoute('r');r.add(dict(type='via',pose=[.4,0,0],pass_radius_m=.02));r.add(dict(type='task',point_id='point_1',pose=[.8,0,0]));r.start({'point_1'},pose=[0,0,0])
  r.observe([.36,0,0],[.36,0,0],1.)
  with self.assertRaisesRegex(ValueError,'unvisited'):r.finish('arrived_stopped',pose=[.36,0,0])
  self.assertEqual(r.mode,'navigation');self.assertEqual(r.index,0)
  r.observe([.39,0,0],[.39,0,0],1.2)
  self.assertEqual(r.finish('arrived_stopped',pose=[.39,0,0])['point_id'],'point_1')
