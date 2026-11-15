import copy
import hashlib,json
import unittest
import numpy as np
from agent.held_battery import HeldBatteryMonitor


IDENTITY={'reference_id':'qualified-fixture','camera':'d405','serial':'262622271422',
          'handeye_sha256':'a'*64,'tcp_sha256':'b'*64,'battery_sha256':'c'*64,'grasp_id':'grasp-fixture'}


def observation(i=1,t=10.,delta=(0.,0.,0.)):
    return dict(identity=IDENTITY.copy(),frame_id=f'f{i}',captured_at=t,presence='present',
        engagement='visually_consistent',pose='matched',image_delta=list(delta),
        observable_axes=['image_x','image_y','image_rotation'],metric_pose=None,
        uncertainty_px=.25,critical_visible=True,reference_sha256='e'*64)


def feedback(t=10.):return dict(captured_at=t,position_raw=-1.,fresh=True)
def certificate(t=10.):return dict(scope='simulation',reference_id=IDENTITY['reference_id'],grasp_id=IDENTITY['grasp_id'],
    issued_at=t,expires_at=t+20.,unobserved_axes_bounded=True,load_validated=True,reference_sha256='e'*64,
    identity_sha256=hashlib.sha256(json.dumps(IDENTITY,sort_keys=True,separators=(',',':')).encode()).hexdigest())


class HeldTests(unittest.TestCase):
    def setUp(self):self.m=HeldBatteryMonitor(IDENTITY)
    def stable(self,cert=True):
        for i,t in enumerate([10.,10.15,10.3]):r=self.m.update(observation(i,t),feedback(t),now=t,carry_certificate=certificate() if cert else None)
        return r
    def test_new_frame_is_not_automatically_stable_or_qualified(self):
        r=self.m.update(observation(),feedback(),now=10.,carry_certificate=certificate())
        self.assertEqual(r['stability'],'unknown');self.assertNotEqual(r['holding_eligibility'],'eligible')
    def test_distinct_stable_visual_frames_with_load_evidence_allow_simulated_carry(self):
        r=self.stable();self.assertEqual(r['stability'],'stable_observed');self.assertEqual(r['holding_eligibility'],'eligible')
        self.assertFalse(r['executable']);self.assertEqual(r['mode'],'offline');self.assertIsNone(r['metric_pose'])
    def test_image_stability_without_load_or_unobserved_axis_evidence_stays_unknown(self):
        r=self.stable(False);self.assertEqual(r['stability'],'stable_observed');self.assertEqual(r['holding_eligibility'],'unknown')
    def test_stable_misgrasp_never_becomes_normal(self):
        for i,t in enumerate([10.,10.15,10.3]):
            o=observation(i,t);o['engagement']='visibly_incorrect';r=self.m.update(o,feedback(t),now=t,carry_certificate=certificate())
        self.assertEqual(r['holding_eligibility'],'blocked')
    def test_slip_is_relative_and_is_not_absorbed_as_new_baseline(self):
        self.stable();r=self.m.update(observation(4,10.4,(8.,0.,0.)),feedback(10.4),now=10.4,carry_certificate=certificate())
        self.assertEqual(r['stability'],'slip_suspected');self.assertEqual(r['holding_eligibility'],'blocked')
        r=self.m.update(observation(5,10.6),feedback(10.6),now=10.6,carry_certificate=certificate())
        self.assertEqual(r['holding_eligibility'],'blocked')
    def test_occlusion_is_unknown_not_empty(self):
        self.stable();o=observation(4,10.4);o.update(critical_visible=False,presence='unknown')
        r=self.m.update(o,feedback(10.4),now=10.4,carry_certificate=certificate());self.assertEqual(r['presence'],'unknown');self.assertEqual(r['holding_eligibility'],'unknown')
    def test_visible_empty_overrides_in_range_gripper(self):
        o=observation();o.update(presence='absent',engagement='visibly_incorrect')
        r=self.m.update(o,feedback(),now=10.,carry_certificate=certificate());self.assertEqual(r['holding_eligibility'],'blocked')
    def test_duplicate_frames_do_not_count_and_do_not_refresh_expiry(self):
        o=observation()
        for t in [10.,10.1,10.2]:r=self.m.update(o,feedback(t),now=t,carry_certificate=certificate())
        self.assertNotEqual(r['stability'],'stable_observed');self.assertEqual(r['valid_until'],10.5)
        r=self.m.update(o,feedback(10.7),now=10.7,carry_certificate=certificate());self.assertNotEqual(r['holding_eligibility'],'eligible')
    def test_future_backward_stale_or_changed_identity_are_not_usable(self):
        for key in ['future','stale','identity','nan']:
            m=HeldBatteryMonitor(IDENTITY);o=observation()
            if key=='future':o['captured_at']=11.
            elif key=='stale':o['captured_at']=8.
            elif key=='identity':o['identity']['handeye_sha256']='d'*64
            else:o['image_delta'][0]=float('nan')
            with self.subTest(key=key):self.assertNotEqual(m.update(o,feedback(),now=10.,carry_certificate=certificate())['holding_eligibility'],'eligible')
    def test_conflicting_gripper_feedback_and_ambiguous_pose_block_or_unknown(self):
        o=observation();f=feedback();f['position_raw']=0.
        self.assertEqual(self.m.update(o,f,now=10.,carry_certificate=certificate())['holding_eligibility'],'blocked')
        o['pose']='ambiguous';self.assertNotEqual(HeldBatteryMonitor(IDENTITY).update(o,feedback(),now=10.,carry_certificate=certificate())['holding_eligibility'],'eligible')
    def test_release_phase_empty_does_not_report_transport_slip_or_completion(self):
        self.stable();o=observation(4,10.4);o['presence']='absent'
        r=self.m.update(o,feedback(10.4),now=10.4,phase='released_verified');self.assertEqual(r['holding_eligibility'],'not_required');self.assertFalse(r['placement_confirmed'])
    def test_live_certificate_cannot_turn_offline_component_into_execution(self):
        c=certificate();c['scope']='real'
        for i,t in enumerate([10.,10.15,10.3]):r=self.m.update(observation(i,t),feedback(t),now=t,carry_certificate=c)
        self.assertNotEqual(r['holding_eligibility'],'eligible');self.assertFalse(r['executable'])
    def test_changed_calibration_or_reference_bytes_invalidate_original_carry_certificate(self):
        for change in ('calibration','reference'):
            identity=IDENTITY.copy()
            if change=='calibration':identity['tcp_sha256']='f'*64
            m=HeldBatteryMonitor(identity)
            for i,t in enumerate([10.,10.15,10.3]):
                o=observation(i,t);o['identity']=identity.copy()
                if change=='reference':o['reference_sha256']='f'*64
                r=m.update(o,feedback(t),now=t,carry_certificate=certificate())
            self.assertNotEqual(r['holding_eligibility'],'eligible',change)
    def test_requested_opening_is_not_transport_slip_or_placement_completion(self):
        self.stable();o=observation(4,10.4);o['presence']='absent'
        r=self.m.update(o,feedback(10.4),now=10.4,phase='release_requested')
        self.assertNotEqual(r['holding_eligibility'],'blocked');self.assertFalse(r['placement_confirmed'])
    def test_new_reference_cannot_rebaseline_same_grasp_even_with_reissued_certificate(self):
        self.stable();o=observation(9,10.4);o['reference_sha256']='f'*64;c=certificate();c['reference_sha256']='f'*64
        r=self.m.update(o,feedback(10.4),now=10.4,carry_certificate=c)
        self.assertNotEqual(r['holding_eligibility'],'eligible')
