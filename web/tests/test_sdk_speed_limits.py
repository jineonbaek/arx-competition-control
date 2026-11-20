from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"navigation/polarbear_adapt/mppi_trial"))
import unittest,math,time
from test_control import Controller,FakeAdapter,configured,Rejected
from effort_budget import bound

class OrderedFakeAdapter(FakeAdapter):
 def chassis_output(self,output):self.calls.append(('chassis_output',output))

class BudgetTests(unittest.TestCase):
 def test_translation_08_and_yaw_08_are_allowed(self):
  self.assertEqual(bound((.8,0.,0.)),(.8,0.,0.))
  self.assertEqual(bound((0.,0.,.8)),(0.,0.,.8))
 def test_vector_and_joint_budget_remain_bounded(self):
  v=bound((2.,2.,2.));self.assertLessEqual(math.hypot(*v[:2]),.8+1e-12);self.assertLessEqual(abs(v[2]),.8+1e-12)

class ControlTests(unittest.TestCase):
 def setUp(self):
  cfg=configured();cfg['body'].update(max_linear_m_s=1.6,max_angular_rad_s=1.8,manual_sdk_axes={'x':.92,'y':1.6},chassis_input_scale=1.)
  self.ctrl=Controller(cfg,OrderedFakeAdapter());self.ctrl.update_body(5.,[0.]*4,0.,0.,0.);self.seq=0
 def send(self,kind,x,y,w):
  self.seq+=1
  return self.ctrl.submit(dict(type=kind,source_id='limit-test',task_id='test',generation=1,seq=self.seq,issued_at=time.time(),ttl_s=.3,x=x,y=y,omega=w),'test')
 def test_navigation_api_accepts_08_but_rejects_over_08(self):
  self.send('chassis.nav_sdk',.8,0.,0.)
  self.send('chassis.nav_sdk',0.,0.,.8)
  with self.assertRaises(Rejected):self.send('chassis.nav_sdk',.801,0.,0.)
 def test_legacy_web_transport_keeps_units_and_delivers_sdk_value(self):
  self.send('chassis',.92/.4,0.,1.8/.4)
  sdk=self.ctrl.chassis.snapshot()['active']['sdk']
  self.assertAlmostEqual(sdk[0],.92);self.assertAlmostEqual(sdk[2],1.8)
  with self.assertRaises(Rejected):self.send('chassis',.921/.4,0.,0.)
 def test_web_snapshot_exposes_official_axes_and_conversion(self):
  limits=self.ctrl.snapshot()['limits'];self.assertEqual(limits['forward'],.92);self.assertEqual(limits['lateral'],1.6);self.assertEqual(limits['chassis_input_to_sdk'],.4)
