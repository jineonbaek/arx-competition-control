import json
from pathlib import Path
import tempfile
import unittest
from agent.cabinet_autonomy import OfflineCabinetRun,CabinetTracker
from agent.tests.test_cabinet_gates import context,evidence
from agent.tests.test_held_battery import IDENTITY,observation,feedback,certificate
from agent.tests.test_held_rgb import fixture
from agent.held_rgb import ReferenceMatcher
from agent.cabinet_simulator import SimulationExecutor


def target(i,t):return dict(camera='d435',frame='local_gravity',scope='simulation',target_id='demo:left',model_sha256='e'*64,
    frame_id=f't{i}',captured_at=t,center_local_m=[.55,-.2,.65],normal_local=[-1.,0.,0.],ambiguous=False)
def checked(c):return dict(ik_clear=True,normal_clear=True,stopping_clear=True,coverage_complete=True,
    min_clearance_mm=40.+c['lift_rad'],joint_margin_deg=20.,retreat_clear=True)


class TrackerTests(unittest.TestCase):
    def test_unique_frames_only_and_stale_retrieval(self):
        tr=CabinetTracker()
        for now in (10.,10.1,10.2):r=tr.update(target(1,10.),now=now)
        self.assertFalse(r['ready'])
        tr.update(target(2,10.15),now=10.2);self.assertTrue(tr.update(target(3,10.3),now=10.3)['ready'])
        self.assertFalse(tr.snapshot(now=13.)['ready'])
    def test_camera_relabel_target_switch_ambiguity_or_jump_are_not_consensus(self):
        for mutation in ['camera','target','ambiguous','jump','time']:
            tr=CabinetTracker();tr.update(target(1,10.),now=10.)
            o=target(2,10.15)
            if mutation=='camera':o['camera']='d405'
            elif mutation=='target':o['target_id']='another'
            elif mutation=='ambiguous':o['ambiguous']=True
            elif mutation=='jump':o['center_local_m'][0]+=.3
            else:o['captured_at']=11.
            self.assertFalse(tr.update(o,now=10.15)['ready'])


class FlowTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.runs=[];self.clock=10.;self.seq=0
    def tearDown(self):
        for r in self.runs:r.close()
    def make(self,**kw):
        initial=kw.pop('initial_state',dict(scope='simulation',lift_rad=5.,base_xy_m=[0.,0.],base_yaw_rad=0.))
        r=OfflineCabinetRun(Path(self.tmp.name)/'run',context(),IDENTITY,now=self.clock,initial_state=initial,**kw);self.runs.append(r);return r
    def prime(self,r,*,with_target=True,matcher=False):
        m=None
        if matcher:
            rgb,empty,battery,jaw=fixture();m=ReferenceMatcher(rgb,battery,jaw,identity=IDENTITY,reference_kind='synthetic',empty_rgb=empty)
        for _ in range(3):
            self.seq+=1;self.clock+=.15
            o=(m.analyze(rgb,frame_id=f'f{self.seq}',captured_at=self.clock) if m else observation(self.seq,self.clock))
            c=certificate(self.clock-.5);c['reference_sha256']=o['reference_sha256']
            r.observe_held(o,feedback(self.clock),carry_certificate=c,now=self.clock)
            if with_target:r.observe_target(target(self.seq,self.clock),now=self.clock)
    def gates(self,r):return evidence(r.context,now=self.clock)
    def dispatch(self,r):return r.step(self.gates(r),now=self.clock,check_candidate=checked)
    def complete(self,r,reply):
        self.clock+=.05;s=reply['segment'];actual=r.snapshot()['actual_state'].copy();selected=r.snapshot()['selected']
        if s['action']=='base_align':actual.update(base_xy_m=selected['base_delta_m'],base_yaw_rad=selected['yaw_rad'])
        if s['action']=='lift_align':actual['lift_rad']=selected['lift_rad']
        r.complete_segment(s['segment_id'],now=self.clock,
            completion=dict(scope='simulation',segment_id=s['segment_id'],actual_state_seq=r.context.state_seq+1,
                actual_pose_verified=True,actual_state=actual,rest_verified=True,completed_at=self.clock))
    def test_rgb_monitor_target_and_gatekeeper_are_connected(self):
        r=self.make();self.prime(r,matcher=True);reply=self.dispatch(r)
        self.assertEqual(reply['next_action'],'base_align');self.assertEqual(reply['segment']['status'],'accepted');self.assertFalse(reply['executable'])
        self.complete(r,reply);self.assertEqual(r.snapshot()['phase'],'reobserve_after_base')
        self.assertEqual(self.dispatch(r)['next_action'],'inspect')
        self.prime(r,matcher=True);reply=self.dispatch(r);self.assertEqual(reply['next_action'],'lift_align')
    def test_unknown_held_state_blocks_base_lift_and_does_not_close_gripper(self):
        r=self.make();self.prime(r);o=observation(100,self.clock);o['critical_visible']=False
        r.observe_held(o,feedback(self.clock),now=self.clock)
        reply=self.dispatch(r);self.assertIn('G09',reply['decision']['blocked']);self.assertEqual(r.executor.commands,[])
    def test_missing_target_can_search_when_other_gates_hold(self):
        r=self.make();self.prime(r,with_target=False);reply=self.dispatch(r);self.assertEqual(reply['next_action'],'search')
    def test_missing_obstacle_evidence_blocks_search_not_diagnostic(self):
        r=self.make();self.prime(r,with_target=False);e=self.gates(r);del e['G08'];reply=r.step(e,now=self.clock,check_candidate=checked)
        self.assertIn('G08',reply['decision']['blocked']);self.assertEqual(reply['next_action'],'inspect');self.assertEqual(r.executor.commands,[])
    def test_slip_after_prime_latches_stop_instead_of_automatic_tightening(self):
        r=self.make();self.prime(r);self.clock+=.1
        r.observe_held(observation(99,self.clock,(10.,0.,0.)),feedback(self.clock),carry_certificate=certificate(self.clock-.5),now=self.clock)
        self.assertEqual(r.snapshot()['status'],'stopping');self.assertEqual(r.executor.commands,[])
    def test_receipt_acceptance_without_actual_completion_is_unknown(self):
        r=self.make();self.prime(r);reply=self.dispatch(r);self.clock+=.1
        r.complete_segment(reply['segment']['segment_id'],now=self.clock,completion=None)
        self.assertEqual(r.snapshot()['status'],'recovery_required');self.assertIsNotNone(r.snapshot()['inflight'])
    def test_partial_insertion_survives_restart_and_blocks_realign(self):
        r=self.make();r.note_possible_insertion();r.close();self.runs.remove(r)
        self.clock+=1;r=self.make();self.prime(r);reply=self.dispatch(r)
        self.assertEqual(r.snapshot()['engagement'],'may_be_inserted');self.assertEqual(reply['next_action'],'inspect');self.assertEqual(r.executor.commands,[])
    def test_manual_pause_not_auto_resumed_by_fresh_observations(self):
        r=self.make();self.prime(r);r.pause(now=self.clock,reason='takeover');self.prime(r)
        reply=self.dispatch(r);self.assertEqual(reply['next_action'],'inspect');self.assertTrue(r.snapshot()['manual_latched'])
    def test_correction_has_cumulative_limit_and_stops_oscillation(self):
        r=self.make();self.assertTrue(r.request_correction(10.));self.assertTrue(r.request_correction(11.));self.assertFalse(r.request_correction(12.))
        self.assertEqual(r.snapshot()['status'],'hold');self.assertFalse(r.request_correction(1.))
    def test_only_one_writer_or_owner_and_borrower_does_not_release_parent(self):
        r=self.make()
        with self.assertRaises(ValueError):self.make()
        sim=SimulationExecutor('sim-run','sim-boot');parent=sim.acquire('competition',now=10.)
        child=OfflineCabinetRun(Path(self.tmp.name)/'child',context(),IDENTITY,now=10.,executor=sim,parent_owner=parent);self.runs.append(child)
        child.close();self.assertEqual(sim.owner,parent)
    def test_insertion_precision_is_not_inherited_from_coarse_alignment(self):
        r=self.make();self.prime(r)
        for expected in ['base_align','lift_align','outside_align']:
            reply=self.dispatch(r);self.assertEqual(reply['next_action'],expected);self.complete(r,reply);self.prime(r)
        e=self.gates(r);del e['G12'];reply=r.step(e,now=self.clock,check_candidate=checked)
        self.assertIn('G12',reply['decision']['blocked']);self.assertEqual(r.snapshot()['engagement'],'outside_verified')
    def test_docking_uses_recorded_current_lift_and_missing_state_is_not_defaulted(self):
        r=self.make(initial_state=dict(scope='simulation',lift_rad=11.,base_xy_m=[0.,0.],base_yaw_rad=0.));self.prime(r);self.dispatch(r)
        self.assertGreaterEqual(r.snapshot()['selected']['lift_rad'],10.)
        r.close();self.runs.remove(r)
        other=OfflineCabinetRun(Path(self.tmp.name)/'missing',context(),IDENTITY,now=self.clock);self.runs.append(other);self.prime(other)
        reply=self.dispatch(other);self.assertEqual(other.executor.commands,[]);self.assertIn('actual',reply['state']['reason'].lower())
    def test_verified_outside_correction_is_bounded_and_does_not_bypass_insert_precision(self):
        r=self.make();self.prime(r)
        for _ in range(3):reply=self.dispatch(r);self.complete(r,reply);self.prime(r)
        e=self.gates(r);e['G12'].details.update(error_upper_mm=2.,correction_error_mm=3.,correction_safe_outside=True)
        reply=r.step(e,now=self.clock,check_candidate=checked)
        self.assertEqual(reply['next_action'],'inspect');self.assertEqual(r.snapshot()['phase'],'outside_align')
        self.assertEqual(r.snapshot()['correction_count'],1)
    def test_release_outcome_unknown_is_persisted_and_never_reissued(self):
        r=self.make();self.prime(r)
        for _ in range(4):reply=self.dispatch(r);self.complete(r,reply);self.prime(r)
        reply=self.dispatch(r);self.assertEqual(reply['next_action'],'release');self.clock+=.1
        r.complete_segment(reply['segment']['segment_id'],now=self.clock,completion=None)
        self.assertEqual(r.snapshot()['release'],'unknown');count=len(r.executor.commands)
        self.prime(r);self.dispatch(r);self.assertEqual(len(r.executor.commands),count)
    def test_explicit_resume_requires_verified_stop_and_fresh_actual_state(self):
        r=self.make();self.prime(r);r.pause(now=self.clock,reason='manual')
        with self.assertRaises(ValueError):r.resume(expected_generation=r.context.generation,actual_state=r.snapshot()['actual_state'],now=self.clock)
        from agent.tests.test_cabinet_simulator import rest
        for i in range(1,4):self.clock+=.15;r.observe_rest(rest(r.executor,i,self.clock),now=self.clock)
        actual=r.snapshot()['actual_state'].copy();actual.update(captured_at=self.clock,actual_state_seq=1,measured_rest_verified=True,unobserved_distance_upper_m=0.,unobserved_rotation_upper_rad=0.)
        old=r.context.generation;r.resume(expected_generation=old,actual_state=actual,now=self.clock)
        self.assertFalse(r.snapshot()['manual_latched']);self.assertGreater(r.context.generation,old);self.assertEqual(self.dispatch(r)['next_action'],'inspect')
    def test_completion_racing_manual_stop_is_recorded_without_restarting_motion(self):
        r=self.make();self.prime(r);reply=self.dispatch(r);r.pause(now=self.clock,reason='manual');self.complete(r,reply)
        self.assertEqual(r.snapshot()['phase'],'reobserve_after_base');self.assertEqual(r.snapshot()['status'],'stopping')
        self.assertEqual(self.dispatch(r)['next_action'],'inspect')
    def test_inflight_unknown_holding_and_expiry_request_stop(self):
        r=self.make();self.prime(r);self.dispatch(r);self.clock+=.1;o=observation(99,self.clock);o['critical_visible']=False
        r.observe_held(o,feedback(self.clock),now=self.clock)
        self.assertTrue(r.executor.stopping);self.assertEqual(r.snapshot()['status'],'stopping')
        r.close();self.runs.remove(r)
        other=OfflineCabinetRun(Path(self.tmp.name)/'expiry',context(),IDENTITY,now=self.clock,
            initial_state=dict(scope='simulation',lift_rad=5.,base_xy_m=[0.,0.],base_yaw_rad=0.));self.runs.append(other)
        self.prime(other);self.dispatch(other);self.clock+=1.;self.dispatch(other);self.assertTrue(other.executor.stopping)
    def test_initial_nonzero_actual_base_is_charged_to_search_budget(self):
        r=self.make(initial_state=dict(scope='simulation',lift_rad=5.,base_xy_m=[.3,0.],base_yaw_rad=0.));self.prime(r,with_target=False)
        reply=self.dispatch(r);self.assertEqual(reply['next_action'],'search');self.assertAlmostEqual(r.snapshot()['budget']['total_distance_m'],.3)
    def test_expected_empty_during_requested_release_waits_for_receipt_without_grip_fault(self):
        r=self.make();self.prime(r)
        for _ in range(4):reply=self.dispatch(r);self.complete(r,reply);self.prime(r)
        self.dispatch(r);self.clock+=.1;o=observation(99,self.clock);o['presence']='absent'
        r.observe_held(o,feedback(self.clock),now=self.clock);self.dispatch(r)
        self.assertFalse(r.executor.stopping);self.assertEqual(r.snapshot()['release'],'release_requested')
    def test_resume_charges_unobserved_motion_upper_bound(self):
        from agent.tests.test_cabinet_simulator import rest
        r=self.make();r.pause(now=self.clock,reason='manual')
        for i in range(1,4):self.clock+=.15;r.observe_rest(rest(r.executor,i,self.clock),now=self.clock)
        actual=r.snapshot()['actual_state'].copy();actual.update(base_xy_m=[.1,0.],captured_at=self.clock,actual_state_seq=1,
            measured_rest_verified=True,unobserved_distance_upper_m=.15,unobserved_rotation_upper_rad=0.)
        r.resume(expected_generation=r.context.generation,actual_state=actual,now=self.clock)
        self.assertAlmostEqual(r.snapshot()['budget']['total_distance_m'],.15)
    def test_late_completion_cannot_bypass_evidence_or_hard_deadline(self):
        r=self.make();self.prime(r);reply=self.dispatch(r);self.clock+=40.;self.complete(r,reply)
        self.assertTrue(r.executor.stopping);self.assertEqual(r.snapshot()['status'],'stopping')
        self.assertEqual(len(r.snapshot()['records']),1);self.assertEqual(self.dispatch(r)['next_action'],'inspect')
    def test_parent_stop_propagates_to_borrowed_child_without_releasing_parent(self):
        sim=SimulationExecutor('sim-run','sim-boot');parent=sim.acquire('competition',now=self.clock)
        child=OfflineCabinetRun(Path(self.tmp.name)/'borrowed',context(),IDENTITY,now=self.clock,executor=sim,parent_owner=parent,
            initial_state=dict(scope='simulation',lift_rad=5.,base_xy_m=[0.,0.],base_yaw_rad=0.));self.runs.append(child)
        self.prime(child);self.dispatch(child);sim.request_stop('cancel',now=self.clock);self.dispatch(child)
        self.assertEqual(child.snapshot()['status'],'stopping');self.assertEqual(sim.owner,parent)
