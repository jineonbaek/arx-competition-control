"""Check every approved recovery family and explicit unsafe boundaries."""
import importlib,sys,types,unittest
from unittest.mock import patch
try:import rclpy,numpy
except ImportError:
 overrides={name:types.ModuleType(name) for name in ('rclpy','numpy') if name not in sys.modules}
 with patch.dict(sys.modules,overrides):policy=importlib.import_module('retained_goal')
else:policy=importlib.import_module('retained_goal')

class RecoveryClassification(unittest.TestCase):
 def test_all_native_recoverable_codes_request_new_safe_path(self):
  for code in range(102,108):
   with self.subTest(code=code):self.assertEqual(policy.recovery_kind(ValueError('mppi_failed status=6 code='+str(code))),'replan')
 def test_ordinary_occupancy_and_path_withdrawal_never_require_operator(self):
  for cause in ('map_sweep_blocked','odom_sweep_blocked','map_footprint_blocked','odom_footprint_blocked','goal_blocked','replan_blocked','terminal_sweep_blocked','path_withdrawn','no_current_path','path_audit_pending'):
   with self.subTest(cause=cause):self.assertEqual(policy.recovery_kind(ValueError(cause+' fixture')),'replan')
 def test_stale_sources_wait_for_inputs_instead_of_ending_goal(self):
  for cause in ('sensor_or_costmap_stale','bottom_obstacles_stale','dual_origin_mapping_stale','tf_missing','motion_pose_stale_or_invalid','motion_pose_changed_during_proof','input outage exceeded .30 seconds'):
   with self.subTest(cause=cause):self.assertEqual(policy.recovery_kind(ValueError(cause)),'inputs')
 def test_native_unavailability_and_compute_overrun_retry(self):
  for cause in ('mppi_unavailable','mppi_rejected_path','mppi_accept_timeout','control_cycle_overrun'):
   with self.subTest(cause=cause):self.assertEqual(policy.recovery_kind(ValueError(cause)),'replan')
 def test_identity_unknown_effort_and_actual_execution_timeout_stay_parked(self):
  for cause in ('task_worker_identity_changed','slam_alignment_jump_resend_goal','localization jump','invalid_sdk_effort','goal_timeout','maintenance_active','execution_gate_closed','mppi_failed code=100','mppi_failed code=101','mppi_failed code=0'):
   with self.subTest(cause=cause):self.assertEqual(policy.recovery_kind(ValueError(cause)),'park')
