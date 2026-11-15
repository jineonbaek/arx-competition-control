import unittest
from agent.cabinet_simulator import SimulationExecutor
from agent.cabinet_gates import Gatekeeper
from agent.tests.test_cabinet_gates import context,evidence,held


def rest(executor,i,t,**kw):
    row=dict(scope='simulation',generation=executor.generation,sample_id=f's{i}',
        axes={axis:{'seq':i,'captured_at':t,'speed':0.} for axis in executor.axes})
    row.update(kw);return row


class SimulatorTests(unittest.TestCase):
    def setUp(self):self.e=SimulationExecutor('sim-run','sim-boot');self.owner=self.e.acquire('standalone',now=10.)
    def permit(self):
        c=context(generation=self.e.generation);return Gatekeeper().evaluate('base_align',evidence(c),c,now=10.3,held=held())
    def stopped(self):
        self.e.request_stop('manual',now=10.4)
        for i,t in enumerate([10.45,10.6,10.75],1):r=self.e.observe_rest(rest(self.e,i,t),now=t)
        return r
    def test_exclusive_owner_and_no_substitute_hardware_transport(self):
        with self.assertRaises(ValueError):self.e.acquire('competition-parent',now=10.2)
        self.assertEqual(self.e.mode,'offline');self.assertFalse(self.e.executable)
    def test_valid_decision_only_records_a_simulated_intent(self):
        r=self.e.submit('segment-1',self.owner,self.permit(),now=10.3)
        self.assertEqual(r['status'],'accepted');self.assertFalse(r['executable']);self.assertEqual(len(self.e.commands),1)
        with self.assertRaises(ValueError):self.e.submit('segment-1',self.owner,self.permit(),now=10.3)
    def test_stale_generation_and_expired_decision_are_rejected(self):
        p=self.permit();self.e.request_stop('manual',now=10.4)
        with self.assertRaises(ValueError):self.e.submit('s',self.owner,p,now=10.4)
        other=SimulationExecutor('sim-run','sim-boot');owner=other.acquire('another',now=10.)
        with self.assertRaises(ValueError):other.submit('s',owner,p,now=12.)
    def test_stop_dispatch_is_independent_of_failed_feedback_or_movement_gates(self):
        r=self.e.request_stop('feedback_lost',now=10.4)
        self.assertEqual(r['status'],'stopping');self.assertEqual(set(r['stop_requested_axes']),set(self.e.axes));self.assertIsNotNone(self.e.owner)
        with self.assertRaises(ValueError):self.e.release(self.owner)
    def test_service_ack_without_rest_and_cached_samples_never_release(self):
        self.e.request_stop('manual',now=10.4);row=rest(self.e,1,10.45)
        for t in [10.45,10.55,10.7]:self.e.observe_rest(row,now=t)
        self.assertFalse(self.e.rest_verified)
        with self.assertRaises(ValueError):self.e.release(self.owner)
    def test_three_new_post_stop_samples_allow_ownership_transfer(self):
        self.assertTrue(self.stopped()['stopped_verified']);self.e.release(self.owner);self.assertIsNone(self.e.owner)
    def test_one_axis_repeated_or_moving_blocks_rest_certificate(self):
        for kind in ['repeated','moving','nan','old']:
            e=SimulationExecutor('r','b');o=e.acquire('operator',now=10.);e.request_stop('manual',now=10.4)
            for i,t in enumerate([10.45,10.6,10.75],1):
                row=rest(e,i,t)
                if kind=='repeated':row['axes']['lift']['seq']=1
                elif kind=='moving':row['axes']['base']['speed']=.1
                elif kind=='nan':row['axes']['right']['speed']=float('nan')
                else:row['axes']['left']['captured_at']=10.2
                e.observe_rest(row,now=t)
            self.assertFalse(e.rest_verified,kind)
    def test_parent_borrower_cannot_release_parent_control(self):
        child=self.e.borrow(self.owner,'place-child');self.assertNotEqual(child,self.owner)
        with self.assertRaises(ValueError):self.e.release(child)
        self.stopped();self.e.release(self.owner)
    def test_unknown_segment_outcome_is_not_replayed_or_cleared_by_rest(self):
        self.e.submit('release-once',self.owner,self.permit(),now=10.3)
        self.e.receipt('release-once','unknown',now=10.4);self.stopped()
        self.assertTrue(self.e.outcome_unknown)
        with self.assertRaises(ValueError):self.e.release(self.owner)
    def test_old_pose_evidence_cannot_be_reused_after_a_completed_segment(self):
        p=self.permit();self.e.submit('s1',self.owner,p,now=10.3);self.e.receipt('s1','completed',now=10.35)
        with self.assertRaises(ValueError):self.e.submit('s2',self.owner,p,now=10.4)
    def test_old_rest_certificate_cannot_transfer_control_after_a_gap(self):
        self.stopped();self.e.request_stop('manual',now=12.)
        with self.assertRaises(ValueError):self.e.release(self.owner)
    def test_widely_separated_fresh_samples_do_not_prove_continuous_rest(self):
        self.e.request_stop('manual',now=10.)
        for i,t in enumerate([10.1,100.,200.],1):self.e.observe_rest(rest(self.e,i,t),now=t)
        self.assertFalse(self.e.rest_verified)
    def test_cached_post_stop_row_after_expiry_does_not_claim_rest_verified(self):
        self.stopped();r=self.e.observe_rest(rest(self.e,3,10.75),now=12.)
        self.assertFalse(r['stopped_verified'])
