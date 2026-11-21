from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "web" / "src")]

from agent.taught_placement import load_taught_route
from fast_visual_level import (
    FastVisualLevel,
    d405_horizontal_correction_target,
    require_loaded_gripper,
    require_gripper_retained,
    validate_lateral_endpoint,
)


class D405HorizontalPureTests(unittest.TestCase):
    def setUp(self):
        self.route = load_taught_route(ROOT / "config" / "taught-placement.json")
        self.tool = np.eye(4); self.tool[:3, 3] = [.60, -.20, .53]
        self.q = np.radians([5., 38., 37., 0., 2., -90.])

    def test_target_translates_only_base_y(self):
        measurement = dict(status="visual_guidance_only", step_mm=2.5,
                           axis_base=[0., 1., 0.])
        target = d405_horizontal_correction_target(self.tool, measurement, limit_mm=3.)
        expected = self.tool.copy(); expected[1, 3] += .0025
        np.testing.assert_allclose(target, expected)
        for bad in (dict(measurement, step_mm=3.1),
                    dict(measurement, axis_base=[1., 0., 0.])):
            with self.assertRaisesRegex(ValueError, "horizontal correction"):
                d405_horizontal_correction_target(self.tool, bad, limit_mm=3.)

    def test_full_target_and_path_bounds_replace_fixed_y_limit(self):
        for sign in (-1,1):
            measurement=dict(status='visual_guidance_only',step_mm=sign*30.4,axis_base=[0.,1.,0.])
            target=d405_horizontal_correction_target(self.tool,measurement)
            self.assertAlmostEqual(target[1,3]-self.tool[1,3],sign*.0304)
            result=validate_lateral_endpoint(self.tool,target,self.q,self.route,y_bounds_mm=(0.,sign*30.4))
            self.assertAlmostEqual(result['delta_y_mm'],sign*30.4)
            beyond=target.copy();beyond[1,3]+=sign*.002
            with self.assertRaisesRegex(ValueError,'authorized Y'):
                validate_lateral_endpoint(self.tool,beyond,self.q,self.route,y_bounds_mm=(0.,sign*30.4))

    def test_endpoint_rejects_x_z_rotation_axis_and_j6_drift(self):
        good = self.tool.copy(); good[1, 3] += .003
        result = validate_lateral_endpoint(self.tool, good, self.q, self.route)
        self.assertAlmostEqual(result["delta_y_mm"], 3.)
        cases = []
        x = good.copy(); x[0, 3] += .0006; cases.append(x)
        z = good.copy(); z[2, 3] += .0006; cases.append(z)
        r = good.copy(); angle = np.radians(.3); r[:3, :3] = [[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]; cases.append(r)
        for end in cases:
            with self.assertRaisesRegex(ValueError, "lateral endpoint"):
                validate_lateral_endpoint(self.tool, end, self.q, self.route)
        bad_q = self.q.copy(); bad_q[5] = np.radians(-86.)
        with self.assertRaisesRegex(ValueError, "J6"):
            validate_lateral_endpoint(self.tool, good, bad_q, self.route)
        y_over = self.tool.copy(); y_over[1, 3] += .004
        with self.assertRaisesRegex(ValueError, "authorized Y"):
            validate_lateral_endpoint(self.tool, y_over, self.q, self.route,
                                      y_bounds_mm=(0., 3.))

    def test_gripper_feedback_is_locked_until_explicit_release(self):
        self.assertTrue(require_loaded_gripper(-1.30))
        with self.assertRaisesRegex(ValueError, "Loaded battery clamp"):
            require_loaded_gripper(0.)
        self.assertTrue(require_gripper_retained(-1.12, -1.30, tolerance=.25))
        with self.assertRaisesRegex(ValueError, "Gripper feedback changed"):
            require_gripper_retained(-.9, -1.30, tolerance=.25)
        with self.assertRaisesRegex(ValueError, "Loaded battery clamp"):
            require_gripper_retained(-.05, -.26, tolerance=.25)


class FakeModel:
    def __init__(self, state): self.state = state; self.reach = .6
    def wrist_transform(self, q, height):
        tool = np.eye(4); tool[0, 3] = float(q[1]); tool[1, 3] = float(q[0]); tool[2, 3] = .53
        return tool
    def check(self, a, b, height): return {"clear": True}


class FakeController:
    def __init__(self):
        self.q = np.radians([0., 38., 37., 0., 2., -90.])
        self.models = {"right": FakeModel(self)}
        self.config = {"arms": {"right": {"joint_limits_deg": [[-100., 100.]] * 6}}}
    def snapshot(self): return {"arms": {"right": {"joints": self.q.tolist()}}}


class D405HorizontalFlowTests(unittest.TestCase):
    def make_level(self):
        level = object.__new__(FastVisualLevel)
        level.route = load_taught_route(ROOT / "config" / "taught-placement.json")
        level.ctrl = FakeController(); level.height = .34; level.limits = {}
        level.tcp = np.eye(4); level.state = {}; level.run = Path(tempfile.mkdtemp())
        level.battery = {"bounds_mm":[[-46., -37.5, 0.], [36., 37.5, 195.]]}
        level.contact = np.array([0., 0., 169.])
        level.save = lambda **kw: level.state.update(kw)
        level.arm_to = lambda target, **kw: setattr(level.ctrl, "q", np.asarray(target, float))
        def execute(path, model, tcp, preobservation, **kwargs):
            level.ctrl.q = np.asarray(path["joint_waypoints_rad"][-1], float)
        level.execute_d405_lateral_path = execute
        return level

    def test_competition_stroke_reports_cross_track_without_interrupting(self):
        import threading
        level=self.make_level();level.lock=threading.RLock()
        level.auto_insert_160=True;level.driver='test';level.guard=lambda:True;level.reached_route=[]
        level.limits=dict(tcp_speed_mm_s=60.,tcp_acceleration_mm_s2=120.)
        level.timeline=types.SimpleNamespace(plan=lambda seconds:None)
        level.ctrl.adapter=types.SimpleNamespace(trajectory_status=lambda side:dict(command_epoch=1,status='running'))
        from scipy.spatial.transform import Rotation
        model=level.ctrl.models['right']
        forward_kinematics=model.wrist_transform
        def pose(joints,height):
            tool=forward_kinematics(joints,height)
            tool[:3,:3]=Rotation.from_euler('z',joints[3]).as_matrix()
            return tool
        model.wrist_transform=pose
        state=level.ctrl.snapshot
        level.ctrl.snapshot=lambda:dict(state(),body={'height':level.height})
        q=level.ctrl.q.copy();end=q.copy();end[1]+=.150
        path=dict(joint_waypoints_rad=[q.tolist(),end.tolist()],timing={'time_from_start_s':[0.,2.]})
        def run(parts,check,**kwargs):
            level.ctrl.q=end.copy();level.ctrl.q[0]+=.00267;level.ctrl.q[3]=np.radians(87.)
            check()
        level.motion=types.SimpleNamespace(arm=run)
        with patch('fast_visual_level.plan_insert',return_value=path):
            level.execute_locked_axis(level.ctrl.models['right'],np.eye(4),np.array([1.,0.,0.]),150.,
                                      locked_rotation=np.eye(3))
        self.assertAlmostEqual(level.state['rotation_drift_deg'],87.)
        self.assertAlmostEqual(level.state['cross_track_mm'],2.67)
        self.assertAlmostEqual(level.state['peak_cross_track_mm'],2.67)

    def test_fresh_reobservation_converges_and_records_each_step(self):
        level = self.make_level(); frames = []
        for stamp in (10., 11.):
            frames.append({"id": np.array(f"f-{stamp}"), "image_time": np.array(stamp),
                           "calibration_sha256": np.array("a" * 64),
                           "joints":level.ctrl.q.copy(),"lift_rad":np.array(level.height)})
        level.capture_pair = lambda timeout_s,**kwargs: (level.run, frames.pop(0))
        observed_after = []
        measurements = iter([
            dict(status="visual_guidance_only", frame_id="f-10", image_time=10., calibration_sha256="a" * 64,
                 axis_base=[0., 1., 0.], target_offset_mm=42.4, raw_error_mm=2., step_mm=2., in_band=False),
            dict(status="visual_guidance_only", frame_id="f-11", image_time=11., calibration_sha256="a" * 64,
                 axis_base=[0., 1., 0.], target_offset_mm=42.4, raw_error_mm=0., step_mm=0., in_band=True),
        ])
        def estimator(frame, **kw):
            observed_after.append(kw["after_image_time"])
            return next(measurements)
        def planner(_planner, model, q, height, tcp, target, limits, **kw):
            end = q.copy(); end[0] = target[1, 3]
            return {"joint_waypoints_rad": [q.tolist(), end.tolist()],
                    "timing":{"time_from_start_s":[0., 1.]}}
        start = level.ctrl.models["right"].wrist_transform(level.ctrl.q, level.height)
        with patch("fast_visual_level.measure_left_edges", side_effect=estimator), \
             patch("fast_visual_level.continuous_correction", side_effect=planner), \
             patch("fast_visual_level.write_horizontal_overlay") as overlay:
            result = level.auto_d405_horizontal_trim(level.ctrl.models["right"], np.eye(4), start)
        self.assertIsNone(observed_after[0]);self.assertGreater(observed_after[1],10.)
        self.assertTrue(result["converged"]); self.assertFalse(result["degraded"])
        self.assertEqual(result["attempt"], 2); self.assertAlmostEqual(result["cumulative_correction_mm"], 2.)
        self.assertAlmostEqual(level.ctrl.q[0], .002)
        self.assertEqual(overlay.call_count, 2)
        for key in ("d405_attempt", "d405_raw_error_mm", "d405_step_mm", "d405_cumulative_correction_mm", "d405_converged", "d405_degraded"):
            self.assertIn(key, level.state)

    def test_loaded_joint_residual_reobserves_instead_of_rejecting_completed_move(self):
        level=self.make_level();model=level.ctrl.models['right']
        level.guard=lambda:True;level.driver='right';level.reached_route=[]
        level.ctrl.adapter=types.SimpleNamespace(trajectory_status=lambda side:{'command_epoch':1})
        level.execute_d405_lateral_path=types.MethodType(FastVisualLevel.execute_d405_lateral_path,level)
        origin=model.wrist_transform(level.ctrl.q,level.height);captures=[];dispatched=[]
        def capture(_,**kwargs):
            captures.append(level.ctrl.q.copy())
            stamp=10. if len(captures)==1 else 12.
            return None,dict(id=str(stamp),image_time=stamp,joints=level.ctrl.q.copy(),lift_rad=level.height)
        def measure(frame,**kw):
            done=frame['image_time']==12.
            if done:self.assertEqual(kw['after_image_time'],11.)
            return dict(frame_id=frame['id'],image_time=frame['image_time'],axis_base=[0.,1.,0.],
                        status='visual_guidance_only',raw_error_mm=0. if done else 2.,in_band=done)
        def plan(_,m,q,h,tcp,target,*args,**kw):
            end=q.copy();end[0]=target[1,3]
            return dict(joint_waypoints_rad=[q.tolist(),end.tolist()],timing={'time_from_start_s':[0.,1.]})
        def execute(segments,*args,**kw):
            dispatched.append(segments)
            level.ctrl.q=np.asarray(segments[-1]['joint_waypoints_rad'][-1]).copy()
            # A completed loaded trajectory can retain joint residual even
            # when the next independent image confirms lateral alignment.
            level.ctrl.q[2]+=np.deg2rad(.8)
        level.capture_pair=capture;level.motion=types.SimpleNamespace(arm=execute)
        with patch('fast_visual_level.measure_left_edges',side_effect=measure), \
             patch('fast_visual_level.write_horizontal_overlay'), \
             patch('fast_visual_level.continuous_correction',side_effect=plan), \
             patch('fast_visual_level.time.time',return_value=11.):
            result=level.auto_d405_horizontal_trim(model,np.eye(4),origin)
        self.assertTrue(result['converged']);self.assertEqual(result['moves'],1)
        self.assertEqual(len(captures),2);self.assertEqual(len(dispatched),1)
        self.assertAlmostEqual(level.state['d405_joint_tracking_max_error_deg'],.8)
        self.assertEqual(level.state['d405_moves'],1)

    def test_completed_move_record_survives_post_move_pose_rejection(self):
        level=self.make_level();model=level.ctrl.models['right']
        origin=model.wrist_transform(level.ctrl.q,level.height)
        level.capture_pair=lambda timeout,**kwargs:(None,dict(id='first',image_time=10.,joints=level.ctrl.q.copy(),lift_rad=level.height))
        def plan(_,m,q,h,tcp,target,*args,**kw):
            end=q.copy();end[0]=target[1,3]
            return dict(joint_waypoints_rad=[q.tolist(),end.tolist()])
        def execute(path,*args,**kw):
            level.ctrl.q=np.asarray(path['joint_waypoints_rad'][-1]).copy()
            level.ctrl.q[0]+=.0025
        level.execute_d405_lateral_path=execute
        measurement=dict(frame_id='first',image_time=10.,axis_base=[0.,1.,0.],
                         status='visual_guidance_only',raw_error_mm=2.,in_band=False)
        with patch('fast_visual_level.measure_left_edges',return_value=measurement), \
             patch('fast_visual_level.write_horizontal_overlay'), \
             patch('fast_visual_level.continuous_correction',side_effect=plan):
            with self.assertRaisesRegex(ValueError,'authorized Y'):
                level.auto_d405_horizontal_trim(model,np.eye(4),origin)
        self.assertEqual(level.state['d405_moves'],1)
        self.assertAlmostEqual(level.state['d405_cumulative_correction_mm'],4.5)

    def test_signed_one_pass_ignores_frame_from_during_motion(self):
        import threading
        for correction in (-12.993461,12.698849,-30.40875,30.40875):
            with self.subTest(correction=correction):
                level=self.make_level();level.cancel=threading.Event();level.guard=lambda:None
                model=level.ctrl.models['right'];origin=model.wrist_transform(level.ctrl.q,level.height)
                exposures=iter([10.,10.5,12.]);seen=[];targets=[]
                def capture(_,**kwargs):
                    stamp=next(exposures)
                    return None, dict(id=str(stamp),image_time=stamp,joints=level.ctrl.q.copy(),
                                      lift_rad=level.height)
                def estimate(frame,**kwargs):
                    self.assertIsNone(kwargs['max_step_mm'])
                    stamp=frame['image_time'];after=kwargs['after_image_time']
                    seen.append((stamp,after))
                    if after is not None and stamp<=after:
                        raise ValueError('older exposure')
                    err=correction if stamp==10. else 0.
                    return dict(status='visual_guidance_only',axis_base=[0.,1.,0.],
                                frame_id=str(stamp),image_time=stamp,raw_error_mm=err,step_mm=err,in_band=err==0.)
                def plan(_,model,q,height,tcp,target,limits,**kw):
                    targets.append(target.copy());end=q.copy();end[0]=target[1,3]
                    return {'joint_waypoints_rad':[q.tolist(),end.tolist()]}
                level.capture_pair=capture
                with patch('fast_visual_level.measure_left_edges',side_effect=estimate), \
                     patch('fast_visual_level.continuous_correction',side_effect=plan), \
                     patch('fast_visual_level.write_horizontal_overlay'),patch('fast_visual_level.time.time',return_value=11.):
                    result=level.auto_d405_horizontal_trim(model,np.eye(4),origin)
                self.assertTrue(result['converged']);self.assertEqual(result['moves'],1)
                self.assertEqual(len(targets),1)
                expected=origin.copy();expected[1,3]+=correction/1000.
                np.testing.assert_allclose(targets[0],expected)
                self.assertEqual(seen,[(10.,None),(10.5,11.),(12.,11.)])

    def test_unavailable_reobserves_until_stopped_and_does_not_move(self):
        import threading
        level=self.make_level();q=level.ctrl.q.copy();calls=[];level.cancel=threading.Event()
        def guard():
            if len(calls)==3:raise ValueError('cancelled')
        level.guard=guard
        def unavailable(timeout,**kwargs):
            calls.append(timeout)
            raise ValueError('D405 offline')
        level.capture_pair=unavailable
        start=level.ctrl.models['right'].wrist_transform(q,level.height)
        with self.assertRaisesRegex(ValueError,'cancelled'):
            level.auto_d405_horizontal_trim(level.ctrl.models['right'],np.eye(4),start)
        self.assertEqual(len(calls),3)
        np.testing.assert_allclose(level.ctrl.q,q)

    def test_legacy_z_limit_does_not_clip_horizontal_target(self):
        import threading
        level=self.make_level()
        level.route=types.SimpleNamespace(**(level.route.__dict__|{'d405_total_limit_mm':1.}))
        level.guard=lambda:True;level.cancel=threading.Event()
        model=level.ctrl.models['right'];start=model.wrist_transform(level.ctrl.q.copy(),level.height)
        frames=iter(range(1,3));moves=[]
        def capture(_,**kwargs):
            stamp=next(frames)
            return None,dict(id=stamp,image_time=stamp,joints=level.ctrl.q.copy(),lift_rad=level.height)
        def measure(f,**kwargs):
            done=f['id']==2
            return dict(frame_id=str(f['id']),image_time=f['id'],axis_base=[0.,1.,0.],status='visual_guidance_only',
                        raw_error_mm=0. if done else 3.,step_mm=0. if done else 3.,in_band=done)
        def plan(_,m,q,h,tcp,target,*args,**kw):
            end=q.copy();end[0]=target[1,3];return {'joint_waypoints_rad':[q.tolist(),end.tolist()]}
        def execute(path,*a,**kw):
            moves.append(kw['y_end_mm']);level.ctrl.q=np.array(path['joint_waypoints_rad'][-1])
        level.capture_pair=capture;level.execute_d405_lateral_path=execute
        with patch('fast_visual_level.measure_left_edges',side_effect=measure),patch('fast_visual_level.write_horizontal_overlay'),patch('fast_visual_level.continuous_correction',side_effect=plan):
            result=level.auto_d405_horizontal_trim(model,np.eye(4),start)
        self.assertTrue(result['converged']);self.assertEqual(result['attempt'],2)
        self.assertEqual(len(moves),1);self.assertAlmostEqual(moves[0],3.)

    def test_planned_path_is_prevalidated_before_any_dispatch(self):
        level = self.make_level(); dispatched = []
        level.motion = types.SimpleNamespace(arm=lambda *args, **kwargs: dispatched.append(args))
        level.ctrl.adapter = types.SimpleNamespace(trajectory_status=lambda side: {
            "driver_instance":"driver", "command_epoch":1})
        level.driver = "driver"; level.guard = lambda: True; level.reached_route = []
        start = level.ctrl.models["right"].wrist_transform(level.ctrl.q, level.height)
        points = [level.ctrl.q.copy(), level.ctrl.q.copy(), level.ctrl.q.copy()]
        points[1][1] = .001
        path = {"joint_waypoints_rad":[p.tolist() for p in points],
                "timing":{"time_from_start_s":[0., .5, 1.]}}
        with self.assertRaisesRegex(ValueError, "lateral endpoint"):
            FastVisualLevel.execute_d405_lateral_path(
                level, path, level.ctrl.models["right"], np.eye(4), start)
        self.assertEqual(dispatched, [])

    def test_joint_interpolation_is_sampled_not_only_its_endpoints(self):
        level = self.make_level(); dispatched = []
        class CurvedModel(FakeModel):
            def wrist_transform(self, q, height):
                tool = super().wrist_transform(q, height)
                tool[0, 3] += .001 * np.sin(np.pi * float(q[0]) / .002)
                return tool
        model = CurvedModel(level.ctrl)
        level.motion = types.SimpleNamespace(arm=lambda *args, **kwargs: dispatched.append(args))
        level.ctrl.adapter = types.SimpleNamespace(trajectory_status=lambda side: {
            "driver_instance":"driver", "command_epoch":1})
        level.driver = "driver"; level.guard = lambda: True; level.reached_route = []
        start_q = level.ctrl.q.copy(); start_q[0] = 0.; level.ctrl.q = start_q
        end_q = start_q.copy(); end_q[0] = .002
        start = model.wrist_transform(start_q, level.height)
        path = {"joint_waypoints_rad":[start_q.tolist(), end_q.tolist()],
                "timing":{"time_from_start_s":[0., 1.]}}
        with self.assertRaisesRegex(ValueError, "lateral endpoint"):
            FastVisualLevel.execute_d405_lateral_path(
                level, path, model, np.eye(4), start)
        self.assertEqual(dispatched, [])

    def test_joint_interpolation_cannot_overshoot_authorized_y(self):
        level = self.make_level(); dispatched = []
        class YCurvedModel(FakeModel):
            def wrist_transform(self, q, height):
                tool = super().wrist_transform(q, height)
                tool[1, 3] += .002 * np.sin(np.pi * float(q[0]) / .002)
                return tool
        model = YCurvedModel(level.ctrl)
        level.motion = types.SimpleNamespace(arm=lambda *args, **kwargs: dispatched.append(args))
        level.ctrl.adapter = types.SimpleNamespace(trajectory_status=lambda side: {
            "driver_instance":"driver", "command_epoch":1})
        level.driver = "driver"; level.guard = lambda: True; level.reached_route = []
        start_q = level.ctrl.q.copy(); start_q[0] = 0.; level.ctrl.q = start_q
        end_q = start_q.copy(); end_q[0] = .002
        start = model.wrist_transform(start_q, level.height)
        path = {"joint_waypoints_rad":[start_q.tolist(), end_q.tolist()],
                "timing":{"time_from_start_s":[0., 1.]}}
        with self.assertRaisesRegex(ValueError, "authorized Y"):
            FastVisualLevel.execute_d405_lateral_path(
                level, path, model, np.eye(4), start, y_start_mm=0., y_end_mm=2.)
        self.assertEqual(dispatched, [])

    def test_segment_endpoints_also_reserve_analytic_gap_margin(self):
        level = self.make_level(); dispatched = []
        base_q = level.ctrl.q.copy()
        class NearLimitModel(FakeModel):
            def wrist_transform(self, q, height):
                tool = super().wrist_transform(q, height)
                tool[0, 3] += .49 * (float(q[2]) - float(base_q[2]))
                return tool
        model = NearLimitModel(level.ctrl)
        level.motion = types.SimpleNamespace(arm=lambda *args, **kwargs: dispatched.append(args))
        level.ctrl.adapter = types.SimpleNamespace(trajectory_status=lambda side: {
            "driver_instance":"driver", "command_epoch":1})
        level.driver = "driver"; level.guard = lambda: True; level.reached_route = []
        end_q = base_q.copy(); end_q[2] += .001
        start = model.wrist_transform(base_q, level.height)
        path = {"joint_waypoints_rad":[base_q.tolist(), end_q.tolist()],
                "timing":{"time_from_start_s":[0., 1.]}}
        with self.assertRaisesRegex(ValueError, "lateral endpoint"):
            FastVisualLevel.execute_d405_lateral_path(
                level, path, model, np.eye(4), start)
        self.assertEqual(dispatched, [])


if __name__ == "__main__": unittest.main()

class T2ContinuityTests(unittest.TestCase):
    def test_clearance_check_at_dispatch_uses_cached_sweep_slack(self):
        from fast_visual_level import bind_approach_sweep
        q=np.array([0.,.01,.45,-.46,-.04,-1.58])
        def state(q,left=None,height=.44):
            return dict(arms=dict(right=dict(joints=q),left=dict(joints=np.zeros(6) if left is None else left)),
                        body=dict(height=height))
        certified=state(q)
        bind=bind_approach_sweep(certified,q,[dict(slack_m=.026,right_radius_m=1.3,left_radius_m=1.)])
        bind(state(q+.0001))
        for changed in (state(q+.02),state(q,left=np.ones(6)*.1),state(q,height=3.5)):
            with self.assertRaises(ValueError):bind(changed)

    def test_in_band_y_accepts_recorded_small_pose_drift_without_extra_move(self):
        from scipy.spatial.transform import Rotation
        fixture=D405HorizontalFlowTests()
        level=fixture.make_level()
        model=level.ctrl.models['right'];fk=model.wrist_transform
        def pose(q,h):
            t=fk(q,h);t[:3,:3]=Rotation.from_euler('z',q[3]).as_matrix();return t
        model.wrist_transform=pose
        lock=pose(level.ctrl.q,level.height)
        frames=iter(range(1,5));goals=[];executions=[]
        def capture(_,**kwargs):
            stamp=next(frames)
            return None,dict(id=stamp,image_time=stamp,joints=level.ctrl.q.copy(),lift_rad=level.height)
        level.capture_pair=capture
        def measure(frame,**kwargs):
            first=frame['id']==1
            return dict(frame_id=str(frame['id']),image_time=frame['image_time'],axis_base=[0.,1.,0.],
                        status='visual_guidance_only',raw_error_mm=3. if first else 0.,step_mm=3. if first else 0.,in_band=not first)
        def planner(_,m,q,h,tcp,target,*args,**kw):
            goals.append(target.copy());end=q.copy();end[0]=target[1,3];end[3]=Rotation.from_matrix(target[:3,:3]).as_rotvec()[2]
            return dict(joint_waypoints_rad=[q.tolist(),end.tolist()])
        def execute(path,*a,**kw):
            level.ctrl.q=np.array(path['joint_waypoints_rad'][-1]);executions.append(True)
            if len(executions)==1:level.ctrl.q[3]+=np.deg2rad(.44537)
        level.execute_d405_lateral_path=execute
        with patch('fast_visual_level.measure_left_edges',side_effect=measure),patch('fast_visual_level.write_horizontal_overlay'),patch('fast_visual_level.continuous_correction',side_effect=planner):
            result=level.auto_d405_horizontal_trim(model,np.eye(4),lock)
        self.assertTrue(result['converged']);self.assertEqual(len(goals),1)
        self.assertAlmostEqual(level.state['d405_measured_pose']['rotation_drift_deg'],.44537,places=4)

    def test_unissued_rail_start_checks_small_loaded_drift_without_replaying(self):
        from fast_visual_level import validate_rail_start
        from docking_motion import ArmStartChanged
        from scipy.spatial.transform import Rotation
        class Model:
            def wrist_transform(self,q,h):
                t=np.eye(4);t[0,3]=q[0];t[:3,:3]=Rotation.from_euler('z',q[3]).as_matrix();return t
            def check(self,a,b,h,**kw):
                return {'clear':True}
        model=Model();q=np.zeros(6);start=dict(joints=q.tolist(),height=.44)
        def pose(x,angle):
            q=np.zeros(6);q[0]=x;q[3]=np.radians(angle)
            return dict(arms={'right':dict(joints=q.tolist())},body={'height':.44})
        validate_rail_start(model,np.eye(4),pose(.00036,.17),start)
        for actual in (pose(.003,.1),pose(.0003,2.)):
            with self.assertRaises(ArmStartChanged):validate_rail_start(model,np.eye(4),actual,start)
        model.check=lambda *a,**kw:{'clear':False}
        with self.assertRaises(ArmStartChanged):validate_rail_start(model,np.eye(4),pose(.00036,.17),start)

    def test_rail_rebind_only_before_native_dispatch_and_no_replay_on_lost_ack(self):
        from unittest.mock import Mock
        from docking_motion import DockingMotion,ArmStartChanged
        for mode,expected_count in [('small',1),('outside',0),('lost_ack',1)]:
            with self.subTest(mode=mode):
                state={'arms':{'right':{'joints':[.003,0.,0.,0.,0.,0.]}},'body':{'height':.44}}
                ctrl=Mock()
                ctrl.snapshot.return_value=state;ctrl.clock.return_value=1.
                ctrl.adapter.trajectory_status.return_value={'driver_instance':'right','command_epoch':5}
                ctrl.poll_trajectory.return_value={'status':'reference_complete'}
                motion=DockingMotion(ctrl,'owner',guard=lambda s,now:True);motion._rest=Mock(return_value=state)
                plan={'joint_waypoints_rad':[[0.]*6,[.2]*6],'time_from_start_s':[0.,2.]}
                sent=[]
                def send(side,command,token,*,preflight,expected_driver,require_lift_settle):
                    self.assertTrue(preflight(state,1.));self.assertEqual(expected_driver['command_epoch'],5)
                    sent.append(command)
                    if mode=='lost_ack':raise IOError('no acknowledgement')
                    return expected_driver
                ctrl.start_trajectory.side_effect=send
                def accept(actual,initial,reference):
                    self.assertEqual(reference,plan['joint_waypoints_rad'][0])
                    if mode=='outside':raise ArmStartChanged('rail connector rejected')
                def run():
                    return motion.arm([plan],lambda:None,expected_start={'joints':[0.]*6,'height':.44},
                                    expected_driver={'driver_instance':'right','command_epoch':5},rail_start_check=accept)
                if mode=='small':run()
                else:
                    with self.assertRaises((ArmStartChanged,IOError)):run()
                self.assertEqual(sent,[plan]*expected_count)

    def test_startup_waits_for_a_single_ready_driver_sample_without_sending(self):
        from fast_visual_level import wait_entry_feedback
        from unittest.mock import Mock
        ctrl=Mock();ctrl.motion_valid.return_value=True
        ctrl.snapshot.return_value={'arms':{'left':{'fresh':True},'right':{'fresh':True}},'body':{'fresh':True},'chassis_active':False}
        ready={'driver_instance':'right','command_epoch':3,'ready':True,'enabled':True,'status':'holding'}
        ctrl.adapter.trajectory_status.side_effect=[None,None,ready]
        notices=[];sleep=Mock()
        state,driver=wait_entry_feedback(ctrl,'owner',lambda:False,notices.append,sleep=sleep)
        self.assertEqual(driver,ready);self.assertEqual(len(notices),1);self.assertEqual(sleep.call_count,2)
        ctrl.start_trajectory.assert_not_called()
        with self.assertRaises(ValueError):wait_entry_feedback(ctrl,'owner',lambda:True,notices.append,sleep=sleep)
