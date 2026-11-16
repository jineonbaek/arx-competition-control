import unittest,numpy as np,time,threading
from fast_visual_level import FastVisualLevel,require_joint_arrival,require_stationary_pose,payload_in_tool,require_grip

class CabinetRetryTest(unittest.TestCase):
    def test_d405_target_uses_bounded_measured_step(self):
        from fast_visual_level import d405_correction_target
        T=np.eye(4);T[:3,3]=[.3,-.1,.6]
        target=d405_correction_target(T,{'status':'visual_guidance_only','step_mm':5.})
        np.testing.assert_allclose(target[:3,3],[.3,-.1,.605])
        with self.assertRaises(ValueError):d405_correction_target(T,{'status':'visual_guidance_only','step_mm':5.01})
        with self.assertRaises(ValueError):d405_correction_target(T,{'status':'invalid','step_mm':1.})
    def test_small_cabinet_margin_is_explicit_and_bounded(self):
        from held_insertion_sweep import valid_outside_margin
        self.assertTrue(valid_outside_margin(.1))
        self.assertFalse(valid_outside_margin(.09))
        self.assertFalse(valid_outside_margin(float('nan')))
    def test_small_up_target_is_bounded(self):
        from fast_visual_level import recovery_tool_target
        T=np.eye(4);T[:3,3]=[.3,-.1,.6]
        target=recovery_tool_target(T,[0,0,1],3.)
        np.testing.assert_allclose(target[:3,3],[.3,-.1,.603])
        np.testing.assert_allclose(target[:3,:3],T[:3,:3])
    def test_continuous_ik_refines_only_branch_jumps(self):
        from fast_visual_level import continuous_correction
        calls=[]
        def planner(*args,**kwargs):
            calls.append(kwargs['translation_step_mm'])
            if len(calls)<3:raise ValueError('Correction IK branch jump or singular configuration')
            return 'path'
        self.assertEqual(continuous_correction(planner,1,2,timing_limits={}), 'path')
        self.assertEqual(calls,[5.,2.,1.])
        with self.assertRaisesRegex(ValueError,'collision'):
            continuous_correction(lambda *a,**k:(_ for _ in ()).throw(ValueError('collision')),1,2,timing_limits={})
    def test_alignment_does_not_command_retreat(self):
        from fast_visual_level import no_retreat_target
        start=np.eye(4);start[:3,3]=[.3,0,.6]
        target=start.copy();target[:3,3]+=[-.012,.003,.004]
        safe=no_retreat_target(start,target,[-1.,0.,0.])
        np.testing.assert_allclose(safe[:3,3],[.3,.003,.604])
        forward=start.copy();forward[0,3]+=.003
        np.testing.assert_allclose(no_retreat_target(start,forward,[-1.,0.,0.]),forward)
    def test_recovery_targets_retreat_then_up_without_rotation(self):
        from fast_visual_level import recovery_tool_target
        T=np.eye(4);T[:3,3]=[.3,-.1,.6]
        retreat=recovery_tool_target(T,[-1,0,0],12.)
        up=recovery_tool_target(retreat,[0,0,1],10.)
        np.testing.assert_allclose(up[:3,3],[.288,-.1,.61])
        np.testing.assert_allclose(up[:3,:3],T[:3,:3])
        with self.assertRaises(ValueError):recovery_tool_target(T,[0,0,0],12.)
    def test_table_override_does_not_override_other_collisions(self):
        from held_insertion_sweep import validate_clearances
        bounds={'table':-1.,'base':20.};limits={'table':.5,'base':10.}
        validate_clearances(bounds,limits,skip_table=True)
        with self.assertRaisesRegex(ValueError,'table'):validate_clearances(bounds,limits,skip_table=False)
        with self.assertRaisesRegex(ValueError,'base'):validate_clearances({'table':1.,'base':0.},limits,skip_table=True)
    def test_rotation_target_keeps_tool_position(self):
        from fast_visual_level import rotated_tool_target
        T=np.eye(4);T[:3,3]=[.3,-.1,.6]
        target=rotated_tool_target(T,[0,1,0],[0,0,1])
        np.testing.assert_allclose(target[:3,3],T[:3,3])
        self.assertAlmostEqual(np.degrees(np.arccos((np.trace(target[:3,:3])-1)/2)),1.5,places=5)
    def test_outside_preflight_retries_only_transient_scene_variation(self):
        from fast_visual_level import preflight_outside
        def scene(x,stamp=100.):
            return ({'corners_base_m':[[x,0,0]]*4,'image_time':stamp},
                    {'front_offset_m':.5,'top_m':.6,'bottom_m':.4})
        readings=iter([scene(0),scene(.006),scene(0),scene(.002,98.),scene(.0025)])
        proof={'lower_mm':{'cabinet_standoff':35.,'arm_standoff':206.,'table':2.},
               'required_mm':{'cabinet_standoff':.5,'arm_standoff':20.,'table':.5}}
        window,table,actual=preflight_outside(lambda:next(readings),lambda w,t:proof,lambda:None,lambda **kw:None,clock=lambda:100.)
        self.assertEqual(window['corners_base_m'][0][0],.002)
        self.assertIs(actual,proof)
        with self.assertRaisesRegex(ValueError,'collision'):
            preflight_outside(lambda:scene(0),lambda w,t:(_ for _ in ()).throw(ValueError('collision')),lambda:None,lambda **kw:None,clock=lambda:100.)
    def test_raise_target_preserves_xy_and_orientation(self):
        from fast_visual_level import raised_tool_target
        T=np.eye(4);T[:3,3]=[.3,-.1,.6]
        target=raised_tool_target(T)
        np.testing.assert_allclose(target[:3,3],[.3,-.1,.61])
        np.testing.assert_allclose(target[:3,:3],T[:3,:3])
        np.testing.assert_allclose(T[:3,3],[.3,-.1,.6])
    def test_scene_and_table_drift_use_their_own_clearance(self):
        from fast_visual_level import validate_scene_drift
        proof={'lower_mm':{'cabinet_standoff':35.,'arm_standoff':206.,'table':1.5},
               'required_mm':{'cabinet_standoff':.5,'arm_standoff':20.,'table':.5}}
        validate_scene_drift(proof,2.4,.14)
        with self.assertRaises(ValueError):validate_scene_drift(proof,2.4,1.1)
        with self.assertRaises(ValueError):validate_scene_drift(proof,35.,.14)
    def test_retries_only_transient_plane_failure(self):
        from fast_visual_level import retry_cabinet_observation
        calls=[]
        def read():
            calls.append(1)
            if len(calls)<3:raise ValueError('红框前沿深度不符合单一平面')
            return 'fresh'
        self.assertEqual(retry_cabinet_observation(read),'fresh')
        self.assertEqual(len(calls),3)
        with self.assertRaisesRegex(ValueError,'other'):
            retry_cabinet_observation(lambda: (_ for _ in ()).throw(ValueError('other')))
class FastLevelTests(unittest.TestCase):
    def test_auto_full_never_loads_saved_cabinet_pose(self):
        import inspect
        source=inspect.getsource(FastVisualLevel.cabinet)+inspect.getsource(FastVisualLevel.auto_full_place)
        self.assertNotIn('taught-cabinet-reference',source)
        self.assertNotIn('reference_source',source)

    def test_auto_full_has_fresh_d435_outer_alignment_stage(self):
        import inspect
        source=inspect.getsource(FastVisualLevel.auto_full_place)
        self.assertIn('d435_outer_align',source)
        self.assertNotIn('taught_observation',source)

    def test_d405_unavailable_stops_without_learned_vertical_bias(self):
        from types import SimpleNamespace
        t=FastVisualLevel(None,None)
        t.route=SimpleNamespace(d405_attempts=2,d405_step_limit_mm=5.,d405_total_limit_mm=15.)
        t.save=lambda **kw:None
        def unavailable():raise ValueError('rim hidden')
        t.capture_pair=unavailable
        with self.assertRaisesRegex(ValueError,'D405 vertical evidence unavailable'):
            t.auto_d405_trim(None,None,None,None,0.)

    def test_release_is_ordered_after_measured_full_depth(self):
        import inspect
        source=inspect.getsource(FastVisualLevel.auto_full_place)
        self.assertLess(source.index("'full_insert'"),source.index("'release_and_return'"))
        insert=inspect.getsource(FastVisualLevel.execute_locked_axis)
        self.assertLess(insert.index('require_joint_arrival'),insert.index("self.save(depth_mm"))

    def test_auto_full_is_allowed_without_release_or_target_parameters(self):
        t=FastVisualLevel(None,None)
        class Ctrl:
            def snapshot(self):
                return dict(motion_reserved=False,competition_reserved=False,chassis_active=False)
        t.ctrl=Ctrl()
        result=t.action(dict(op='auto_full'))
        self.assertEqual(result['status'],'running')
        self.assertEqual(result['mode'],'auto_full')
        t.cancel.set()
        t.thread.join(1.)
        for data in [dict(op='release'),dict(op='auto_full',depth=10),dict(op='auto_full',confirm=True)]:
            with self.assertRaises(ValueError):t.action(data)

    def test_d405_target_allows_five_mm_step_and_rejects_larger(self):
        from fast_visual_level import d405_correction_target
        T=np.eye(4);T[:3,3]=[.3,-.1,.6]
        np.testing.assert_allclose(d405_correction_target(T,{'status':'visual_guidance_only','step_mm':5.})[:3,3],[.3,-.1,.605])
        with self.assertRaises(ValueError):d405_correction_target(T,{'status':'visual_guidance_only','step_mm':5.01})
    def test_grip_monitor_keeps_observing_while_planning_thread_is_busy(self):
        from fast_visual_level import ContinuousMonitor
        observed=threading.Event();calls=[]
        def sample():
            calls.append(1)
            if len(calls)>=4:observed.set()
        monitor=ContinuousMonitor(sample)
        try:self.assertTrue(observed.wait(.8))
        finally:monitor.close()
        self.assertGreaterEqual(len(calls),4)
    def test_background_grip_monitor_errors_are_not_silenced(self):
        from fast_visual_level import ContinuousMonitor
        def sample():raise ValueError('lost object')
        monitor=ContinuousMonitor(sample)
        try:
            self.assertTrue(monitor.failed.wait(.8))
            with self.assertRaisesRegex(ValueError,'lost object'):monitor.check()
        finally:monitor.close()
    def test_depth_restart_without_exposure_is_not_ready_for_motion(self):
        from fast_visual_level import camera_ready
        self.assertFalse(camera_ready(dict(state='starting',captured_at=None),100.))
        self.assertFalse(camera_ready(dict(state='online',captured_at=98.),100.))
        self.assertTrue(camera_ready(dict(state='online',captured_at=99.9),100.))
    def test_depth_capture_keeps_monitoring_grip_until_read_only_request_returns(self):
        from fast_visual_level import guarded_read
        observed=[];ready=threading.Event()
        def read():
            self.assertTrue(ready.wait(1.));return 'exposure'
        def guard():
            observed.append(time.monotonic())
            if len(observed)>=4:ready.set()
        self.assertEqual(guarded_read(read,guard,timeout_s=1.),'exposure')
        self.assertGreaterEqual(len(observed),4)
    def test_read_only_capture_wait_stops_on_guard_failure(self):
        from fast_visual_level import guarded_read
        ready=threading.Event();calls=[]
        def guard():
            calls.append(1)
            if len(calls)>1:raise ValueError('grip changed')
        try:
            with self.assertRaisesRegex(ValueError,'grip changed'):
                guarded_read(lambda:ready.wait(1.),guard,timeout_s=1.)
        finally:ready.set()
    def test_visual_attachment_not_nominal_tcp_controls_payload_geometry(self):
        tool=np.eye(4);tool[0,3]=.3
        body=np.eye(4);body[:3,3]=[.5,.1,.6]
        local=payload_in_tool(np.array([[0.,0.,0.],[10.,20.,30.]]),np.linalg.inv(tool)@body)
        np.testing.assert_allclose(local[0],[.2,.1,.6]);np.testing.assert_allclose(local[-1],[.21,.12,.63])
    def test_every_inference_poll_rejects_deviation_before_return_motion(self):
        s=dict(arms=dict(right=dict(joints=[0.]*6)),body=dict(height=1.))
        require_stationary_pose(s,np.zeros(6),1.,8,8)
        s['arms']['right']['joints'][0]=.01
        with self.assertRaises(ValueError):require_stationary_pose(s,np.zeros(6),1.,8,8)
        s['arms']['right']['joints'][0]=0.
        with self.assertRaises(ValueError):require_stationary_pose(s,np.zeros(6),1.,9,8)
    def test_unknown_held_object_evidence_blocks_even_when_camera_is_online(self):
        for evidence in [dict(stable=False,status='unknown'),dict(stable=False,status='slip_suspected')]:
            with self.assertRaises(ValueError):require_grip(evidence)
        require_grip(dict(stable=True,status='stable_visible'))
    def test_unachieved_transport_cannot_advance_to_lift(self):
        target=np.radians([0.,0.,0.,0.,0.,-90.])
        with self.assertRaises(ValueError):require_joint_arrival(np.zeros(6),target,2.)
        require_joint_arrival(np.radians([0.,0.,0.,0.,0.,-89.]),target,2.)
    def test_small_wrist_step_cannot_be_confirmed_by_stationarity(self):
        target=np.radians([0.,0.,0.,1.5,0.,0.])
        with self.assertRaises(ValueError):require_joint_arrival(np.zeros(6),target,.6)
    def test_api_rejects_arbitrary_targets_and_release(self):
        t=FastVisualLevel(None,None)
        for data in [dict(op='release'),dict(op='start',height=10),dict(op='start',joints=[0]*6)]:
            with self.assertRaises(ValueError):t.action(data)

    def test_runtime_files_are_resolved_from_repository_root(self):
        from fast_visual_level import ROOT,WORK
        self.assertIs(WORK,ROOT)

    def test_d435_estimator_uses_current_python_and_checkout(self):
        import sys
        from fast_visual_level import ROOT,d435_estimator_command
        command=d435_estimator_command()
        self.assertEqual(command,[sys.executable,str(ROOT/'scripts/estimate-held-d435.py')])
        self.assertNotIn('/home/arx/',(ROOT/'scripts/estimate-held-d435.py').read_text())
