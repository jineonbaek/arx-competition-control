import json,tempfile,threading,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from agent.taught_placement import load_preobservation_waypoint
from web.src.handeye_column import NOMINAL_RAD_PER_M
from fast_visual_level import FastVisualLevel,require_stationary_pose
from docking_motion import DockingMotion
class HeightIndependent(unittest.TestCase):
 def test_recorded_waypoint_rebases_to_actual_height_without_admission_window(self):
  tool=np.eye(4);tool[:3,3]=[.6,-.2,.5]
  payload=dict(version=1,arm='right',joints_rad=[0.]*6,wrist_base_mm_deg=[600.,-200.,500.,0.,0.,0.],tool_base_matrix=tool.tolist(),lift_rad=0.,gripper_command_target=0,captured_at=1.,source='test',source_hashes={'config/right-grip-tcp.json':'a'*64})
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'point.json';p.write_text(json.dumps(payload));result=load_preobservation_waypoint(p,3.)
  self.assertAlmostEqual(result['tool_base_matrix'][2][3],.5+3./NOMINAL_RAD_PER_M)
  self.assertAlmostEqual(result['wrist_base_mm_deg'][2],500.+3000./NOMINAL_RAD_PER_M);self.assertEqual(result['lift_rad'],3.)
 def test_pose_sampling_and_aux_binding_have_no_lift_delta_threshold(self):
  state=dict(arms={'right':dict(joints=[0.]*6)},body=dict(height=3.))
  require_stationary_pose(state,np.zeros(6),0.,4,4)
  m=DockingMotion(object(),object(),guard=lambda *a:True,check_lift=False)
  m._binding(state,dict(joints=[0.]*6,height=0.),None)
  state['arms']['right']['joints'][0]=.2
  with self.assertRaises(ValueError):m._binding(state,dict(joints=[0.]*6,height=0.),None)
 def test_real_placement_guard_uses_measured_height_without_lift_change_gate(self):
  state=dict(arms={side:dict(fresh=True,joints=[0.]*6,gripper=-.7) for side in ('left','right')},body=dict(fresh=True,height=3.,waist=0.,head_yaw=0.,head_pitch=0.),chassis_active=False)
  ctrl=SimpleNamespace(snapshot=lambda:state,motion_valid=lambda token:True,adapter=SimpleNamespace(trajectory_status=lambda side:dict(driver_instance='driver',ready=True,enabled=True)))
  task=FastVisualLevel(ctrl,None);task.token=object();task.cancel=threading.Event();task.left_q=np.zeros(6);task.driver='driver';task.initializing=False;task.raw=-.7;task.height=0.;task.mode='auto_insert_160';task.auto_insert_160=True;task.d435_only=False;task.motion=SimpleNamespace(active=None);task.body_fixed=dict(waist=0.,head_yaw=0.,head_pitch=0.);task.inserting=False
  self.assertTrue(task.guard());self.assertEqual(task.height,3.)
 def test_arm_entry_rechecks_payload_at_dispatch_and_measured_motion_height(self):
  state=dict(arms={side:dict(fresh=True,joints=[0.]*6,gripper=-.7) for side in ('left','right')},body=dict(fresh=True,height=3.,waist=0.,head_yaw=0.,head_pitch=0.),chassis_active=False)
  ctrl=SimpleNamespace(snapshot=lambda:state,motion_valid=lambda token:True,adapter=SimpleNamespace(trajectory_status=lambda side:dict(driver_instance='driver',ready=True,enabled=True)))
  task=FastVisualLevel(ctrl,None);task.token=object();task.cancel=threading.Event();task.left_q=np.zeros(6);task.driver='driver';task.initializing=False;task.raw=-.7;task.height=0.;task.mode='auto_insert_160';task.auto_insert_160=True;task.d435_only=False;task.motion=SimpleNamespace(active='arm');task.body_fixed=dict(waist=0.,head_yaw=0.,head_pitch=0.);task.inserting=False;task.local_target=np.ones(6);task.geometry_tick=0.
  checks=[];task.local_sweep=lambda a,b,s,**kw:checks.append((s['body']['height'],kw))
  self.assertTrue(task.guard());self.assertEqual(checks[0][0],3.);self.assertFalse(checks[0][1]['check_guard'])
if __name__=='__main__':unittest.main()
