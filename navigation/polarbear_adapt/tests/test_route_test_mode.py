import unittest,uuid,math
from mission_route import MissionRoute
from route_test_mode import test_definition,normal_definition,Backoff
class TestMode(unittest.TestCase):
 def definition(self):
  r=MissionRoute('m')
  for kind,x,tag in [('special',0,None),('special',1,None),('task',2,'point_1'),('special',3,None),('task',4,'point_2'),('special',5,None)]:
   p=dict(type=kind,pose=[x,0,0])
   if tag:p['point_id']=tag
   r.add(p)
  return r.definition()
 def test_working_copy_round_trips_and_keeps_official_tasks(self):
  d=self.definition();t=test_definition(d,True)
  self.assertFalse(any(p['type']=='task' for p in t['points']));self.assertEqual(t['points'][2]['test_task_id'],'point_1');self.assertEqual(normal_definition(t),d);self.assertEqual(test_definition(t,False),d)
 def test_special_chain_stops_before_test_target_and_restarts_from_actual_pose(self):
  r=MissionRoute('m');r.load(test_definition(self.definition(),True));r.start(pose=[0,0,0])
  self.assertEqual(r.target_index,1)
  r.finish('arrived_stopped',pose=[1,0,0]);self.assertEqual(r.target_index,2);self.assertEqual(r.mode,'direct')
  r.finish('arrived_stopped',pose=[1.9,0,0]);self.assertEqual(r.mode,'direct');self.assertEqual(r.direct_source['pose'],[1.9,0,0]);self.assertEqual(r.target_index,3)
 def test_via_then_test_target_keeps_separate_direct_approach(self):
  r=MissionRoute('m');r.add(dict(type='via',pose=[1,0,0]));r.add(dict(type='special',test_task_id='point_1',pose=[2,0,0]));r.start(pose=[0,0,0]);self.assertEqual(r.target_index,0)
 def test_mixed_real_tasks_and_test_markers_cannot_start(self):
  r=MissionRoute('m');r.add(dict(type='special',test_task_id='point_1',pose=[1,0,0]));r.add(dict(type='task',point_id='point_2',pose=[2,0,0]))
  with self.assertRaisesRegex(ValueError,'mixed'):r.start({'point_2'},pose=[0,0,0])
 def test_backoff_stops_once_and_never_corrects(self):
  b=Backoff([2,3,math.pi/2]);self.assertEqual(b.command([2,3,math.pi/2]),(-.14,0.,0.));self.assertEqual(b.command([2,2.9,math.pi/2]),(0.,0.,0.));self.assertEqual(b.command([2,2.91,math.pi/2]),(0.,0.,0.))
 def test_backoff_rejects_over_limit_and_wrong_direction(self):
  for p in [[-.201,0,0],[.02,0,0],[0,.06,0],[0,0,.2]]:
   with self.subTest(p=p),self.assertRaises(ValueError):Backoff([0,0,0]).command(p)
if __name__=='__main__':unittest.main()
