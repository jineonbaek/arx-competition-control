"""Server task uses actual ControllerBridge with fake motor transport only."""
import copy,tempfile,unittest
import numpy as np
import test_cabinet_controller_bridge as fixture
from cabinet_execution import CabinetExecutionTask

class ExecutionTests(unittest.TestCase):
 def setUp(self):
  self.f=fixture.BridgeTests();self.f.setUp();self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
  self.xyz=np.zeros(3)
  class Model:
   def wrist_transform(inner,q,lift):
    t=np.eye(4);t[:3,3]=self.xyz;return t
  self.f.controller.models['right']=Model()
  self.bundle=dict(asset_hash='a'*64,held_identity_hash='b'*64,tcp_matrix=np.eye(4).tolist(),segments=[dict(phase='outside_align',plan=self.f.plan,target_tcp_mm=[0.,0.,0.])])
  self.task=self.make()
 def make(self,provider=True):
  return CabinetExecutionTask(self.f.controller,None,directory=self.tmp.name,
    planner=lambda model:copy.deepcopy(self.bundle),physical_provider=self.f.provider if provider else None,
    bridge_policy=self.f.policy,auto_worker=False)
 def start(self):return self.task.action({'op':'start','model':'demo'})
 def settle(self):
  self.task.tick();self.f.running();self.f.driver['status']='reference_complete'
  for _ in range(8):self.f.now+=.1;self.f.refresh([.01]*6);self.task.tick()
  return self.task.snapshot()
 def test_real_bridge_dispatches_only_trusted_internal_plan(self):
  self.start();self.task.tick();self.assertEqual(self.f.adapter.calls[0],('trajectory',self.f.plan));self.assertTrue(self.f.controller.snapshot()['motion_reserved'])
 def test_missing_provider_rejects_before_any_transport_or_reservation(self):
  self.task=self.make(False);self.start();r=self.task.tick()
  self.assertEqual(r['status'],'blocked');self.assertIn('physical provider',r['reason']);self.assertEqual(self.f.adapter.calls,[]);self.assertIsNone(self.f.controller.motion_owner)
 def test_outside_actual_fk_allows_five_mm_without_changing_joint_gates(self):
  self.xyz[:]=[.005,0,0];self.start();r=self.settle();self.assertEqual(r['status'],'completed_hold');self.assertFalse(r['battery_placed'])
 def test_outside_actual_fk_rejects_more_than_five_mm_and_never_starts_next(self):
  self.bundle['segments'].append(copy.deepcopy(self.bundle['segments'][0]));self.xyz[:]=[.0051,0,0];self.start();r=self.settle()
  self.assertEqual(r['status'],'recovery_required');self.assertEqual(sum(c[0]=='trajectory' for c in self.f.adapter.calls),1)
 def test_browser_cannot_supply_targets_provider_tolerance_or_approval(self):
  for d in [{'op':'start','model':'demo','target':[0]*6},{'op':'start','model':'demo','tolerance_mm':5},{'op':'start','model':'demo','approval':True},{'op':'enable'},{'op':'start','model':'unknown'}]:
   with self.assertRaises(ValueError):self.task.action(d)
  self.assertEqual(self.f.adapter.calls,[])
 def test_cancel_before_dispatch_latches_and_never_advances(self):
  self.start();self.task.action({'op':'cancel'});self.task.tick();self.assertEqual(self.f.adapter.calls,[]);self.assertEqual(self.task.snapshot()['status'],'cancelled')
 def test_planning_rejection_is_reported_without_motor_traffic(self):
  def blocked(model):raise ValueError('aperture collision')
  self.task.planner=blocked;r=self.start();self.assertEqual(r['status'],'blocked');self.assertIn('collision',r['reason']);self.assertEqual(self.f.adapter.calls,[])
 def test_restart_after_unfinished_task_requires_recovery(self):
  self.start();other=self.make();self.assertEqual(other.snapshot()['status'],'recovery_required')
  with self.assertRaises(ValueError):other.action({'op':'start','model':'demo'})

 def test_saved_non_object_receipt_enters_recovery_instead_of_crashing_server(self):
  from pathlib import Path
  for raw in ('[]','null','"bad"'):
   Path(self.tmp.name,'receipt.json').write_text(raw)
   self.assertEqual(self.make().snapshot()['status'],'recovery_required')
 def test_grip_change_after_planning_does_not_become_new_accepted_baseline(self):
  self.start();self.f.refresh(grip=-.7);r=self.task.tick()
  self.assertEqual(r['status'],'blocked');self.assertEqual(self.f.adapter.calls,[])
 def test_manual_command_after_planning_invalidates_whole_run(self):
  self.start();self.f.controller.command_epoch+=1;r=self.task.tick()
  self.assertEqual(r['status'],'blocked');self.assertEqual(self.f.adapter.calls,[])
 def test_unknown_result_remains_recovery_when_cancelled(self):
  self.start();self.task.tick();self.task.bridge.outcome_unknown=True;self.task.bridge.status='recovery_required'
  self.task.action({'op':'cancel'});r=self.task.tick();self.assertEqual(r['status'],'recovery_required')
 def test_production_planner_keeps_requested_visual_outside_target(self):
  from pathlib import Path
  from types import SimpleNamespace
  from unittest.mock import patch
  import json
  root=Path(self.tmp.name);(root/'config').mkdir();(root/'config/right-grip-tcp.json').write_text(json.dumps({'T_wrist_from_tool':np.eye(4).tolist()}))
  p=dict(tcp_pre_base_mm=[10.,20.,30.],tcp_end_base_mm=[20.,20.,30.],R_base_from_tool=np.eye(3).tolist())
  self.task.preview=SimpleNamespace(root=root,action=lambda data,**kwargs:dict(status='localized',plan=p,validity={'valid':True},source_hashes={},snapshot_id='frame'))
  path=dict(joint_waypoints_rad=[[0.]*6,[.01]*6],timing={'time_from_start_s':[0.,1.]})
  self.f.controller.models['right'].reach=1.
  with patch('agent.grasp_path.plan_correction',return_value=path),patch('agent.grasp_path.plan_insert',return_value=path):
   bundle=self.task._plan('demo')
  self.assertEqual(bundle['segments'][0]['target_tcp_mm'],p['tcp_pre_base_mm'])

 def test_no_physical_provider_skips_expensive_ik_after_current_geometry_check(self):
  from types import SimpleNamespace
  from unittest.mock import patch
  self.task.provider=None
  self.task.preview=SimpleNamespace(action=lambda data,**kwargs:dict(status='localized',plan={'nominal_cad_clearance_mm':2},validity={'valid':True}))
  with patch('agent.grasp_path.plan_correction') as solve:
   with self.assertRaisesRegex(ValueError,'physical provider'):self.task._plan('demo')
   solve.assert_not_called()
