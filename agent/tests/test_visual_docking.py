import copy
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from agent.docking_fit import evaluate_fit
from agent.docking_grip import GripTracker
from agent.visual_docking import DockingRun


def geometry():
    tool=np.eye(4);tool[:3,3]=[.3,0,.6]
    window=dict(center_base_m=[.6,0,.6],outward_normal_base=[-1,0,0],
                left_axis_base=[0,1,0],up_axis_base=[0,0,1])
    slot=dict(size_mm=[169.6,79.],center_mm=[0,0],
              outline_mm=[[-84.8,-39.5],[84.8,-39.5],[84.8,39.5],[-84.8,39.5]])
    battery=dict(bounds_mm=[[-41.175,-37.5,0],[41.175,37.5,195.28]])
    return tool,window,slot,battery,[0,0,169.28]


class FitTests(unittest.TestCase):
    def test_centered_full_body_has_only_two_mm_vertical_clearance(self):
        fit=evaluate_fit(*geometry())
        self.assertAlmostEqual(fit['nominal_clearance_mm'],2.,places=4)
        self.assertTrue(fit['qualified_fit'])

    def test_half_mm_lateral_error_acceptable_but_three_mm_vertical_is_not(self):
        args=list(geometry());args[0][1,3]+=.0005
        self.assertTrue(evaluate_fit(*args)['qualified_fit'])
        args[0][2,3]+=.003
        self.assertFalse(evaluate_fit(*args)['nominal_fit'])

    def test_tilt_long_body_cannot_be_accepted_from_tip_alone(self):
        args=list(geometry());args[0][:3,:3]=Rotation.from_euler('y',3,degrees=True).as_matrix()
        self.assertFalse(evaluate_fit(*args)['qualified_fit'])

    def test_uncertainty_is_separate_from_nominal_fit(self):
        fit=evaluate_fit(*geometry(),uncertainty_mm=4.2)
        self.assertTrue(fit['nominal_fit']);self.assertFalse(fit['qualified_fit'])
        self.assertLess(fit['available_clearance_mm'],0)

    def test_bad_transform_or_negative_uncertainty_rejected(self):
        args=list(geometry());args[0][0,0]=2
        with self.assertRaises(ValueError):evaluate_fit(*args)
        with self.assertRaises(ValueError):evaluate_fit(*geometry(),uncertainty_mm=-1)
    def test_concave_aperture_cannot_be_qualified_from_corners_alone(self):
        args=list(geometry());args[2]['outline_mm']=[[-85,-40],[85,-40],[85,40],[5,40],[5,0],[-5,0],[-5,40],[-85,40]]
        with self.assertRaises(ValueError):evaluate_fit(*args)
    def test_real_rounded_aperture_tolerates_stl_float_rounding(self):
        import json
        args=list(geometry())
        profiles=json.loads((Path(__file__).resolve().parents[2]/'config/cabinet-profiles.json').read_text())
        args[2]=profiles['demo']['openings'][0]
        self.assertTrue(evaluate_fit(*args)['nominal_fit'])


def image(dx=0,angle=0):
    rgb=np.full((240,320,3),220,np.uint8)
    rect=cv2.boxPoints(((160+dx,110),(90,120),angle)).astype(np.int32)
    cv2.fillConvexPoly(rgb,rect,(25,25,25))
    return rgb


class GripTests(unittest.TestCase):
    def test_gradual_recorded_lighting_change_keeps_same_fixed_grasp_reference(self):
        p=Path(__file__).parent/'fixtures/d405-illumination.npz'
        with np.load(p) as data:a,b=data['reference'],data['changed']
        m=GripTracker();m.bind(a,1.,.874,now=1.);reference=m.reference
        for i in range(1,31):
            frame=cv2.addWeighted(a,1-i/30,b,i/30,0)
            result=m.update(frame,1+i*.1,.874,now=1+i*.1)
        self.assertTrue(result['stable'],result)
        self.assertIs(m.reference,reference)

    def test_slow_cumulative_slip_is_detected_without_frame_rebaseline(self):
        m=GripTracker();m.bind(image(),1.,.89,now=1.)
        for i in range(1,13):result=m.update(image(dx=i),1+i*.1,.89,now=1+i*.1)
        self.assertEqual(result['status'],'slip_suspected')

    def test_independent_frames_establish_stability_not_one_repeated_frame(self):
        m=GripTracker();m.bind(image(),1.,.89,now=1.)
        self.assertFalse(m.update(image(),1.,.89,now=1.)['stable'])
        m.update(image(),1.1,.89,now=1.1)
        self.assertTrue(m.update(image(),1.25,.89,now=1.25)['stable'])

    def test_slip_is_latched_and_does_not_automatically_rebaseline(self):
        m=GripTracker();m.bind(image(),1.,.89,now=1.)
        self.assertEqual(m.update(image(15),1.1,.89,now=1.1)['status'],'slip_suspected')
        self.assertEqual(m.update(image(),1.3,.89,now=1.3)['status'],'slip_suspected')

    def test_missing_stale_and_gripper_drift_cannot_be_stable(self):
        m=GripTracker();m.bind(image(),1.,.89,now=1.)
        self.assertFalse(m.update(image(),1.1,.89,now=5.)['stable'])
        self.assertFalse(m.update(image(),1.2,1.1,now=1.2)['stable'])
        with self.assertRaises(ValueError):GripTracker().bind(np.full_like(image(),220),1.,.89,now=1.)

    def test_rotation_detected_even_if_centroid_does_not_move(self):
        m=GripTracker();m.bind(image(),1.,.89,now=1.)
        self.assertEqual(m.update(image(angle=8),1.1,.89,now=1.1)['status'],'slip_suspected')
    def test_small_connected_shadow_does_not_rotate_unchanged_battery_texture(self):
        changed=image();cv2.rectangle(changed,(204,60),(225,120),(25,25,25),-1)
        m=GripTracker();m.bind(image(),1.,.89,now=1.)
        m.update(changed,1.1,.89,now=1.1)
        self.assertTrue(m.update(changed,1.25,.89,now=1.25)['stable'])


class SimulatedBackend:
    """Deterministic physical-state simulator at the backend boundary."""
    def __init__(self,strategy='arm',failed=None):
        self.strategy=strategy;self.failed=failed;self.cancel=False;self.seq=0
        self.aligned=False;self.inside=False;self.actions=[];self.holds=0;self.now=100.
    def cancelled(self):return self.cancel
    def observe(self):
        self.seq+=1;self.now+=.05
        return dict(id=str(self.seq),captured_at=self.now,driver_id='driver1',
                    grip_stable=True,at_entry=self.aligned,outside=not self.inside,
                    fit=dict(qualified_fit=self.aligned,available_clearance_mm=2. if self.aligned else -4.))
    def plan(self,phase,obs,*,reverse_of=None):
        return dict(id=phase+str(self.seq),observation_id=obs['id'],driver_id=obs['driver_id'],
                    phase=phase,strategy=self.strategy if phase=='align' else 'arm',
                    checked_sweep=True,measured_feedback=True,max_duration_s=30.,
                    fixed_z=phase in ('insert','withdraw'),reverse_of=reverse_of,
                    gripper_action='retain')
    def execute(self,plan,check):
        check();phase=plan['phase'];self.actions.append(phase);self.now+=.5
        if self.failed==phase:raise TimeoutError('Uncertain physical outcome')
        if phase=='align':self.aligned=True
        if phase=='insert':self.inside=True
        if phase=='withdraw':self.inside=False
        return dict(plan_id=plan['id'],driver_id='driver1',measured_complete=True,
                    gripper_released=False,fixed_z=plan['fixed_z'],
                    engagement='inside' if self.inside else 'outside')
    def hold(self):self.holds+=1


class RunTests(unittest.TestCase):
    def run_task(self,b,**kwargs):return DockingRun(b,clock=lambda:b.now,**kwargs).run()
    def test_one_run_aligns_inserts_and_withdraws_without_release(self):
        b=SimulatedBackend();r=self.run_task(b)
        self.assertEqual(r['status'],'held_outside')
        self.assertEqual(b.actions,['align','insert','withdraw'])
        self.assertFalse(b.inside);self.assertEqual(b.holds,1)
        self.assertFalse(r['released'])
    def test_base_and_lift_are_strategies_not_separate_manual_sessions(self):
        for strategy in ('base','lift'):
            b=SimulatedBackend(strategy);self.assertEqual(self.run_task(b)['status'],'held_outside')
    def test_uncertain_insert_never_automatically_withdraws_or_retries(self):
        b=SimulatedBackend(failed='insert');r=self.run_task(b)
        self.assertEqual(r['status'],'recovery_required');self.assertEqual(b.actions,['align','insert'])
    def test_stale_frame_or_changed_driver_prevents_new_motion(self):
        for key,value in [('captured_at',90.),('driver_id','other')]:
            b=SimulatedBackend();original=b.observe
            def observe():
                r=original()
                if b.seq>1:r[key]=value
                return r
            b.observe=observe
            self.assertNotEqual(self.run_task(b)['status'],'held_outside')
            self.assertNotIn('insert',b.actions)
    def test_cancel_without_new_camera_frame_still_holds(self):
        b=SimulatedBackend();b.cancel=True
        b.observe=lambda:(_ for _ in ()).throw(AssertionError('Must not need image to cancel'))
        self.assertEqual(self.run_task(b)['status'],'cancelled');self.assertEqual(b.holds,1)
    def test_no_fit_progress_ends_bounded_correction(self):
        b=SimulatedBackend();original=b.execute
        def execute(p,c):
            r=original(p,c);b.aligned=False;return r
        b.execute=execute
        self.assertEqual(self.run_task(b)['status'],'recovery_required')
        self.assertLessEqual(b.actions.count('align'),2);self.assertNotIn('insert',b.actions)
    def test_invalid_plan_does_not_dispatch(self):
        for field,value in [('checked_sweep',False),('measured_feedback',False),('gripper_action','open')]:
            b=SimulatedBackend();original=b.plan
            def plan(*a,**kw):return original(*a,**kw)|{field:value}
            b.plan=plan;self.run_task(b);self.assertEqual(b.actions,[])
    def test_base_insertion_requires_fixed_z_and_measured_receipt(self):
        b=SimulatedBackend();original=b.plan
        def plan(*a,**kw):
            p=original(*a,**kw)
            if p['phase']=='insert':p.update(strategy='base',fixed_z=False)
            return p
        b.plan=plan;self.run_task(b);self.assertNotIn('insert',b.actions)
    def test_saved_inflight_state_is_never_replayed(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'receipt.json';b=SimulatedBackend(failed='insert')
            self.run_task(b,journal=path)
            fresh=SimulatedBackend();r=self.run_task(fresh,journal=path)
            self.assertEqual(r['status'],'recovery_required');self.assertEqual(fresh.actions,[])
    def test_plan_that_outlives_its_camera_observation_is_not_dispatched(self):
        b=SimulatedBackend();original=b.plan
        def plan(*a,**kw):b.now+=1.;return original(*a,**kw)
        b.plan=plan;r=DockingRun(b,clock=lambda:b.now).run()
        self.assertNotEqual(r['status'],'held_outside');self.assertEqual(b.actions,[])
    def test_whole_outside_path_can_contain_multiple_checked_segments(self):
        b=SimulatedBackend();original=b.plan
        def plan(*a,**kw):
            r=original(*a,**kw)
            if r['phase']=='align':r['max_duration_s']=150.
            return r
        b.plan=plan;self.assertEqual(self.run_task(b)['status'],'held_outside')
    def test_duplicate_plan_id_cannot_be_dispatched_in_a_later_phase(self):
        b=SimulatedBackend();original=b.plan
        b.plan=lambda *a,**kw:original(*a,**kw)|dict(id='reused')
        self.run_task(b);self.assertEqual(b.actions,['align'])
    def test_nonadjacent_frame_id_is_not_reused_for_withdrawal(self):
        b=SimulatedBackend();original=b.observe
        def observe():
            r=original()
            if b.seq==3:r['id']='1'
            return r
        b.observe=observe;self.run_task(b)
        self.assertEqual(b.actions,['align','insert'])
    def test_new_id_with_pre_motion_capture_time_is_not_post_motion_evidence(self):
        b=SimulatedBackend();original=b.observe
        def observe():
            r=original()
            if b.seq>1:r['captured_at']=b.now-.2
            return r
        b.observe=observe;self.run_task(b)
        self.assertEqual(b.actions,['align'])


if __name__=='__main__':unittest.main()
