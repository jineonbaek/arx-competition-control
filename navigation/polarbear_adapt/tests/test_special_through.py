import unittest,numpy as np
from special_segments import StraightChain
from mission_route import MissionRoute
class SpecialThrough(unittest.TestCase):
 def test_near_special_retains_cruise_and_terminal_is_nonzero(self):
  chain=StraightChain([0.,0.,0.],[0.,0.,0.],[dict(id='a',type='special',pose=[1.,0.,0.])],continuous=True,terminal_precision=False)
  a=chain.step([.85,0.,0.],[.85,0.,0.],1.,1.,.05,[.6,0.,0.]);self.assertGreater(a.command[0],.5)
  b=chain.step([.91,0.,0.],[.91,0.,0.],1.1,1.1,.1,a.command);self.assertTrue(b.terminal_near);self.assertGreater(b.command[0],.5)
 def test_final_precision_slows_without_zero(self):
  chain=StraightChain([0.,0.,0.],[0.,0.,0.],[dict(id='a',type='special',pose=[1.,0.,0.])],continuous=True)
  a=chain.step([.97,0.,0.],[.97,0.,0.],1.,1.,.1,[.14,0.,0.])
  self.assertTrue(a.terminal_near);self.assertGreater(a.command[0],0.);self.assertLess(a.command[0],.3)
 def test_bend_does_not_apply_arrival_slowdown(self):
  ps=[dict(id='a',type='special',pose=[1.,0.,0.]),dict(id='b',type='special',pose=[1.,1.,0.])]
  chain=StraightChain([0.,0.,0.],[0.,0.,0.],ps,continuous=True,terminal_precision=False)
  a=chain.step([.85,0.,0.],[.85,0.,0.],1.,1.,.05,[.6,0.,0.]);self.assertGreater(np.linalg.norm(a.command[:2]),.5)
 def test_special_chain_includes_following_task_without_stop_boundary(self):
  r=MissionRoute('m')
  for x in [dict(id='a',type='special',pose=[1.,0.,0.]),dict(id='b',type='special',pose=[2.,0.,0.]),dict(id='t',type='task',point_id='point_1',pose=[3.,0.,0.])]:r.add(x)
  target=r.start(['point_1'],pose=[0.,0.,0.],direct_start=True);self.assertEqual(target['id'],'t');self.assertEqual(r.mode,'task_direct');self.assertEqual([x['id'] for x in r.direct_points()],['a','b','t'])
  r.advance_direct('a');r.advance_direct('b');self.assertEqual(r.index,2);self.assertTrue(r.running)
 def test_t2_entry_special_does_not_end_before_task(self):
  r=MissionRoute('m')
  for p in [dict(id='a',type='special',pose=[0.,0.,0.]),dict(id='t',type='task',point_id='point_2',pose=[1.,0.,0.])]:r.add(p)
  self.assertEqual(r.start_t2(['point_2'],[-1.,0.,0.])['id'],'t');self.assertEqual(r.mode,'task_direct')

 def test_final_heading_must_align_before_continuation(self):
  c=StraightChain([0.,0.,0.],[0.,0.,0.],[dict(id='p',type='special',pose=[1.,0.,0.])],continuous=True)
  x=c.step([.97,0.,.12],[.97,0.,.12],1.,1.,.1,[0.,0.,0.])
  self.assertFalse(x.terminal_near);self.assertLess(x.command[2],0.)
 def test_test_marker_is_the_slow_boundary_not_preceding_special(self):
  r=MissionRoute('map')
  r.add(dict(id='a',type='special',pose=[1.,0.,0.]));r.add(dict(id='t',type='special',test_task_id='point_2',pose=[2.,0.,0.]))
  self.assertEqual(r.start([],pose=[0.,0.,0.],direct_start=True)['id'],'t')
  self.assertEqual([p['id'] for p in r.direct_points()],['a','t'])
