import math,unittest
import numpy as np
from mission_route import MissionRoute,point
try:
 from special_segments import StraightChain
except ImportError:StraightChain=None

def p(kind,x=0.,y=0.,name=None):
 v=dict(type=kind,pose=[x,y,0.])
 if name is not None:v['name']=name
 return v

class SpecialRoutes(unittest.TestCase):
 def route(self,*points):
  r=MissionRoute('room')
  for q in points:r.add(q)
  return r
 def test_adjacent_specials_near_source_start_direct_without_navigation(self):
  r=self.route(p('special'),p('special',1.));goal=r.start(pose=[0.,0.,0.])
  self.assertEqual(r.mode,'direct');self.assertEqual(goal['pose'],[1.,0.,0.]);self.assertEqual(r.index,1)
  self.assertEqual(len(r.direct_points()),1)
 def test_distant_or_nonadjacent_specials_keep_normal_navigation(self):
  for points in [(p('special',1.),p('special',2.)),(p('special'),p('via',.5),p('special',1.))]:
   r=self.route(*points);r.start(pose=[0.,0.,0.]);self.assertEqual(r.mode,'navigation');self.assertEqual(r.index,0)
 def test_normal_arrival_at_source_starts_direct_chain(self):
  r=self.route(p('special',1.),p('special',2.));r.start(pose=[0.,0.,0.])
  q=r.finish('arrived_stopped',pose=[1.,0.,0.]);self.assertEqual(q['pose'][0],2.);self.assertEqual(r.mode,'direct')
 def test_chain_intermediate_advances_once_and_terminal_needs_stop(self):
  r=self.route(p('special'),p('special',1.),p('special',2.),p('stop',3.));r.start(pose=[0.,0.,0.])
  points=r.direct_points();r.advance_direct(points[0]['id']);self.assertEqual(r.index,2);self.assertTrue(r.running)
  with self.assertRaises(ValueError):r.advance_direct(points[0]['id'])
  with self.assertRaises(ValueError):r.advance_direct(points[1]['id'])
  q=r.finish('arrived_stopped',pose=[2.,0.,0.]);self.assertEqual(q['pose'][0],3.);self.assertEqual(r.mode,'navigation')
 def test_old_stops_never_become_direct_and_schema2_names_roundtrip(self):
  r=self.route(p('stop'),p('stop',1.));old=r.definition();old['schema_version']=1
  s=MissionRoute('room');s.load(old);s.start(pose=[0.,0.,0.]);self.assertEqual(s.mode,'navigation')
  r=self.route(p('special',name='入口'),p('via',1.));s=MissionRoute('room');s.load(r.definition());self.assertEqual(s.points[0]['name'],'入口')
 def test_named_tasks_do_not_change_identity_and_legacy_load_is_draft(self):
  r=MissionRoute('room');r.add(dict(type='task',pose=[0.,0.,0.],point_id='point_1',name='电池作业甲'))
  d=r.definition();self.assertEqual(d['schema_version'],2)
  s=MissionRoute('room');s.load(d);self.assertEqual(s.points[0]['name'],'电池作业甲');self.assertEqual(s.points[0]['point_id'],'point_1');self.assertFalse(s.running)
 def test_task_release_uses_actual_returned_pose_for_direct_next_segment(self):
  for x,mode in [(0.,'direct'),(.5,'direct')]:
   r=MissionRoute('room');r.add(dict(type='task',pose=[0.,0.,0.],point_id='point_1'));r.add(p('special'));r.add(p('special',1.));r.start({'point_1'},pose=[0.,0.,0.]);r.finish('arrived_stopped')
   self.assertEqual(r.can_enter_direct([x,0.,0.],after_task=True),mode=='direct');r.release_task(pose=[x,0.,0.]);self.assertEqual(r.mode,mode);self.assertEqual(r.direct_source['pose'],[x,0.,0.])
 def test_invalid_names_do_not_enter_route(self):
  for name in (12,'line\nnext','x'*65):
   with self.assertRaises(ValueError):point(p('special',name=name))

class Straight(unittest.TestCase):
 def controller(self,targets=None,entry_map=(0.,0.,0.),entry_odom=(0.,0.,0.)):
  self.assertIsNotNone(StraightChain,'straight controller missing')
  targets=targets or [dict(id='b',type='special',pose=[1.,0.,0.])]
  return StraightChain(entry_map,entry_odom,targets)
 def test_six_directions_and_effort_budget(self):
  from mppi_trial.effort_budget import load
  for goal in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,-1)):
   c=self.controller([dict(id='b',type='special',pose=[*goal,0.])]);r=c.step([0,0,0],[0,0,0],1.,1.,.05,[0,0,0])
   self.assertGreater(np.dot(r.command[:2],goal),0.);self.assertLessEqual(load(r.command),1.+1e-9)
   self.assertEqual(r.command[2],0.)
 def test_map_to_odom_rotation_and_entry_heading_held(self):
  c=self.controller(entry_map=(0.,0.,math.pi/2),entry_odom=(2.,3.,0.))
  r=c.step([0.,0.,math.pi/2],[2.,3.,0.],1.,1.,.05,[0,0,0]);self.assertLess(r.command[1],0.);self.assertAlmostEqual(r.command[0],0.,places=6)
 def test_intermediate_pass_does_not_program_zero_command(self):
  c=self.controller([dict(id='b',pose=[.2,0,0]),dict(id='c',pose=[1.,0,0])]);c.step([0,0,0],[0,0,0],1.,1.,.05,[0,0,0])
  r=c.step([.11,0,0],[.11,0,0],1.3,1.3,.05,[.1,0,0]);self.assertEqual(r.passed_ids,['b']);self.assertGreater(r.command[0],0.);self.assertFalse(r.terminal_near)
 def test_terminal_is_position_only_and_inside_10cm(self):
  c=self.controller();r=c.step([.95,0,1.],[.95,0,1.],1.,1.,.05,[.1,0,0]);self.assertTrue(r.terminal_near);self.assertEqual(tuple(r.command),(0.,0.,0.))
 def test_authority_source_uses_the_shared_650ms_deadline(self):
  c=self.controller();r=c.step([0,0,0],[0,0,0],1.,1.5,.05,[0,0,0]);self.assertGreater(r.command[0],0.)
 def test_repeated_source_stamp_cannot_consume_point(self):
  c=self.controller([dict(id='b',pose=[.2,0,0]),dict(id='c',pose=[1.,0,0])]);c.step([0,0,0],[0,0,0],1.,1.,.05,[0,0,0])
  with self.assertRaises(ValueError):c.step([.15,0,0],[.15,0,0],1.,1.05,.05,[0,0,0])
 def test_map_jump_odom_gap_and_nonfinite_cannot_report_arrival(self):
  for m,o,t,now in [([1.,0,0],[0,0,0],1.1,1.1),([1.,0,0],[1.,0,0],2.,2.),([float('nan'),0,0],[0,0,0],1.1,1.1),([1.,0,0],[1.,0,0],1.1,2.)]:
   c=self.controller();c.step([0,0,0],[0,0,0],1.,1.,.05,[0,0,0])
   with self.assertRaises(ValueError):c.step(m,o,t,now,.05,[0,0,0])
 def test_direct_sdk_is_not_throttled_by_ordinary_si_governor(self):
  c=self.controller();c.step([0,0,0],[0,0,0],1.,1.,.05,[.2,0,0])
  r=c.step([.04,0,0],[.04,0,0],1.1,1.1,.05,[.2,0,0]);self.assertGreater(r.command[0],.2);self.assertLessEqual(r.command[0],.6);self.assertEqual(r.command[1:],(0.,0.))

if __name__=='__main__':unittest.main()
