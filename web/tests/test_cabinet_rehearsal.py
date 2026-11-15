import copy,json,sys,tempfile,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
try:
 from cabinet_rehearsal import CabinetRehearsal,validate_bundle,trajectory_digest
except ImportError:
 CabinetRehearsal=None;validate_bundle=None

def state():
 return {'motion_reserved':False,'chassis_active':False,'command_epoch':7,'body':dict(fresh=True,height=10.,waist=0.,head_yaw=0.,head_pitch=0.),'arms':{s:dict(fresh=True,joints=[0.]*6,gripper=-.04) for s in ['left','right']}}
def bundle():
 b=dict(tcp_matrix=__import__('numpy').eye(4).tolist(),insertion_depth_mm=30.,run_id='empty-test',created_at=time.time(),expires_at=time.time()+120,mode='empty_30mm',initial=state(),source_hashes={'tcp':'a'*64,'scene':'b'*64},certificate=dict(continuous_sweep_checked=True,continuous_lower_mm=10.1,nominal_closed_jaw_only=True),stages=[dict(name='outside',plan=dict(interpolation='segment_quintic_scalar',time_from_start_s=[0.,2.],joint_waypoints_rad=[[0.]*6,[.01]*6])),dict(name='insert',plan=dict(interpolation='segment_quintic_scalar',time_from_start_s=[0.,2.],joint_waypoints_rad=[[.01]*6,[.02]*6])),dict(name='retract',plan=dict(interpolation='segment_quintic_scalar',time_from_start_s=[0.,2.],joint_waypoints_rad=[[.02]*6,[.01]*6]))])
 b['certificate']['trajectory_sha256']=trajectory_digest(b['stages']);return b
class Model:
 def wrist_transform(self,q,lift):
  import numpy as np
  m=np.eye(4);m[0,3]=q[0]*.01;return m
class Controller:
 def __init__(self):self.s=state();self.calls=[];self.phase=None;self.owner=None;self.models={'right':Model()}
 def snapshot(self):
  for a in self.s['arms'].values():a['timestamp']=time.time()
  self.s['body']['timestamp']=time.time()
  return copy.deepcopy(self.s)
 def reserve_motion(self,epoch):
  assert epoch==7 and not self.s['motion_reserved'];self.owner=object();self.s['motion_reserved']=True;self.calls.append('reserve');return self.owner
 def motion_valid(self,t):return self.owner is t
 def start_trajectory(self,side,plan,token,*,preflight):
  assert token is self.owner and preflight(self.snapshot(),0);self.phase=copy.deepcopy(plan);self.calls.append(('trajectory',side,copy.deepcopy(plan)))
 def poll_trajectory(self,token,*,preflight):
  assert preflight(self.snapshot(),0);self.s['arms']['right']['joints']=self.phase['joint_waypoints_rad'][-1];return {'status':'reference_complete'}
 def submit(self,data,token):self.calls.append(('stop',data));self.phase=None
 def release_motion(self,token):self.calls.append('release');self.s['motion_reserved']=False;self.owner=None
class RehearsalTests(unittest.TestCase):
 def test_implementation_available(self):self.assertIsNotNone(CabinetRehearsal)
 def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.c=Controller()
 def tearDown(self):self.tmp.cleanup()
 def instance(self,b=None,approved=True):
  b=b or bundle();self.paths={'tcp':self.root/'tcp.json','scene':self.root/'frame.npz'}
  for k,p in self.paths.items():p.write_bytes(json.dumps({'T_wrist_from_tool':__import__('numpy').eye(4).tolist()}).encode() if k=='tcp' else k.encode());b['source_hashes'][k]=__import__('hashlib').sha256(p.read_bytes()).hexdigest()
  (self.root/'bundle.json').write_text(json.dumps(b));approval={'run_id':b['run_id'],'bundle_sha256':__import__('hashlib').sha256((self.root/'bundle.json').read_bytes()).hexdigest(),'expires_at':b['expires_at']} if approved else None
  if approval:(self.root/'approval.json').write_text(json.dumps(approval))
  return CabinetRehearsal(self.c,self.root,source_paths=self.paths,poll_s=.001,arrival_samples=2,arrival_interval_s=.001,arrival_timeout_s=.05)
 def test_missing_approval_cannot_reserve_or_dispatch(self):
  x=self.instance(approved=False)
  with self.assertRaises(ValueError):x.action({'op':'start','run_id':'empty-test'})
  self.assertEqual(self.c.calls,[])
 def test_modified_bundle_after_approval_is_rejected(self):
  x=self.instance();b=bundle();b['stages'][0]['plan']['joint_waypoints_rad'][1][0]=.02;(self.root/'bundle.json').write_text(json.dumps(b))
  with self.assertRaises(ValueError):x.action({'op':'start','run_id':'empty-test'})
  self.assertEqual(self.c.calls,[])
 def test_only_bound_plan_id_is_accepted(self):
  x=self.instance()
  for d in [{'op':'start','run_id':'other'},{'op':'start','run_id':'empty-test','plan':{} }]:
   with self.assertRaises(ValueError):x.action(d)
  self.assertEqual(self.c.calls,[])
 def test_bad_bundle_does_not_pass_execution_gate(self):
  for change in ['expired','uncertified','long','discontinuous','nan']:
   b=bundle()
   if change=='expired':b['expires_at']=time.time()-1
   if change=='uncertified':b['certificate']['continuous_sweep_checked']=False
   if change=='long':b['stages'][0]['plan']['time_from_start_s'][-1]=61.
   if change=='discontinuous':b['stages'][1]['plan']['joint_waypoints_rad'][0][0]=.5
   if change=='nan':b['stages'][0]['plan']['joint_waypoints_rad'][0][0]=float('nan')
   with self.subTest(change=change),self.assertRaises(ValueError):validate_bundle(b,time.time())
 def test_moved_left_lift_right_or_gripper_cannot_start(self):
  for key in ['left','lift','right','gripper','chassis']:
   self.c=Controller();x=self.instance()
   if key in ['left','right']:self.c.s['arms'][key]['joints'][0]=.1
   if key=='lift':self.c.s['body']['height']=11.
   if key=='gripper':self.c.s['arms']['right']['gripper']=-1.
   if key=='chassis':self.c.s['chassis_active']=True
   with self.subTest(key=key),self.assertRaises(ValueError):x.action({'op':'start','run_id':'empty-test'})
   self.assertEqual(self.c.calls,[])
 def test_success_dispatches_only_quintic_right_arm_and_measured_arrival(self):
  x=self.instance();x.action({'op':'start','run_id':'empty-test'});self.assertTrue(x.wait(2));r=x.snapshot();self.assertEqual(r['status'],'completed');self.assertFalse(r['battery_placed']);self.assertEqual(len([c for c in self.c.calls if isinstance(c,tuple) and c[0]=='trajectory']),3);self.assertIn('release',self.c.calls)
 def test_run_is_not_replayed(self):
  x=self.instance();x.action({'op':'start','run_id':'empty-test'});self.assertTrue(x.wait(2));before=copy.deepcopy(self.c.calls)
  with self.assertRaises(ValueError):self.instance().action({'op':'start','run_id':'empty-test'})
  self.assertEqual(self.c.calls,before)
 def test_drift_during_motion_stops_and_does_not_start_next_stage(self):
  original=self.c.poll_trajectory
  def drift(token,*,preflight):self.c.s['body']['height']=11.;return original(token,preflight=preflight)
  self.c.poll_trajectory=drift;x=self.instance();x.action({'op':'start','run_id':'empty-test'});self.assertTrue(x.wait(2));self.assertEqual(x.snapshot()['status'],'stopped');self.assertTrue(any(isinstance(c,tuple) and c[0]=='stop' for c in self.c.calls));self.assertEqual(len([c for c in self.c.calls if isinstance(c,tuple) and c[0]=='trajectory']),1)
 def test_nonfinite_live_body_cannot_start(self):
  x=self.instance();self.c.s['body']['height']=float('nan')
  with self.assertRaises(ValueError):x.action({'op':'start','run_id':'empty-test'})
  self.assertEqual(self.c.calls,[])
 def test_source_replacement_during_run_stops_before_next_stage(self):
  original=self.c.poll_trajectory
  def changed(token,*,preflight):self.paths['tcp'].write_bytes(b'new TCP');return original(token,preflight=preflight)
  self.c.poll_trajectory=changed;x=self.instance();x.action({'op':'start','run_id':'empty-test'});self.assertTrue(x.wait(2));self.assertEqual(x.snapshot()['status'],'stopped');self.assertEqual(len([c for c in self.c.calls if isinstance(c,tuple) and c[0]=='trajectory']),1)
 def test_reference_complete_without_measured_arrival_is_not_success(self):
  def incomplete(token,*,preflight):return {'status':'reference_complete'}
  self.c.poll_trajectory=incomplete;x=self.instance();x.action({'op':'start','run_id':'empty-test'});self.assertTrue(x.wait(2));self.assertEqual(x.snapshot()['status'],'stopped');self.assertFalse(x.snapshot()['battery_placed'])
 def test_live_tracking_deviation_stops(self):
  import numpy as np
  class SensitiveModel:
   def wrist_transform(self,q,lift):m=np.eye(4);m[0,3]=q[0]*.3;return m
  self.c.models['right']=SensitiveModel();original=self.c.poll_trajectory
  def deviation(token,*,preflight):self.c.s['arms']['right']['joints']=[.009]*6;return original(token,preflight=preflight)
  self.c.poll_trajectory=deviation;x=self.instance();x.action({'op':'start','run_id':'empty-test'});self.assertTrue(x.wait(2));self.assertEqual(x.snapshot()['status'],'stopped')
 def test_cancellation_stops_before_next_stage(self):
  from threading import Event
  entered=Event();release=Event()
  def waiting(token,*,preflight):entered.set();release.wait(1);return {'status':'running'}
  self.c.poll_trajectory=waiting;x=self.instance();x.action({'op':'start','run_id':'empty-test'});self.assertTrue(entered.wait(1));x.action({'op':'cancel'});release.set();self.assertTrue(x.wait(2));self.assertEqual(x.snapshot()['status'],'stopped');self.assertEqual(len([c for c in self.c.calls if isinstance(c,tuple) and c[0]=='trajectory']),1)
 def test_cached_packet_cannot_prove_measured_stop(self):
  x=self.instance();cached=self.c.snapshot();self.c.snapshot=lambda:copy.deepcopy(cached);self.assertFalse(x._arrived([0.]*6))
 def test_same_metadata_but_changed_bytes_fail_start_guard(self):
  import os
  x=self.instance();x.bundle=x._load('empty-test');p=self.paths['scene'];stamp=p.stat();p.write_bytes(b'SCENE');os.utime(p,ns=(stamp.st_atime_ns,stamp.st_mtime_ns));self.assertFalse(x._guard(self.c.snapshot(),0,starting=True))
 def test_parse_and_approval_use_identical_bundle_bytes(self):
  from unittest.mock import patch
  x=self.instance();b=json.loads((self.root/'bundle.json').read_text());a=copy.deepcopy(b);a['insertion_depth_mm']=10.;original=Path.read_text
  def swapped(path,*args,**kwargs):
   if path==self.root/'bundle.json':return json.dumps(a)
   return original(path,*args,**kwargs)
  with patch.object(Path,'read_text',swapped):self.assertEqual(x._load('empty-test')['insertion_depth_mm'],b['insertion_depth_mm'])
 def test_initial_motion_must_settle_before_any_trajectory(self):
  original=self.c.snapshot
  def moving():
   if self.c.owner is not None and self.c.phase is None:self.c.s['arms']['right']['joints'][0]+=.001
   return original()
  self.c.snapshot=moving;x=self.instance();x.action({'op':'start','run_id':'empty-test'});self.assertTrue(x.wait(2));self.assertFalse(any(isinstance(c,tuple) and c[0]=='trajectory' for c in self.c.calls));self.assertNotEqual(x.snapshot()['status'],'completed')
 def test_tool_rotation_deviation_stops_even_if_tcp_position_matches(self):
  import numpy as np
  from scipy.spatial.transform import Rotation
  class RotatingModel:
   def wrist_transform(self,q,lift):m=np.eye(4);m[:3,:3]=Rotation.from_euler('x',q[5]*2).as_matrix();return m
  self.c.models['right']=RotatingModel();original=self.c.poll_trajectory
  def deviation(token,*,preflight):self.c.s['arms']['right']['joints']=[.009]*6;return original(token,preflight=preflight)
  self.c.poll_trajectory=deviation;x=self.instance();x.action({'op':'start','run_id':'empty-test'});self.assertTrue(x.wait(2));self.assertEqual(x.snapshot()['status'],'stopped')
