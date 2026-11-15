import copy
import unittest
from agent.cabinet_gates import Context, Evidence, Gatekeeper, identity_hash
from agent.tests.test_held_battery import IDENTITY,observation,feedback,certificate
from agent.held_battery import HeldBatteryMonitor


def context(**kw):
    d=dict(run_id='sim-run',boot_id='sim-boot',generation=1,asset_hash='d'*64,
           held_identity_hash=identity_hash(IDENTITY),engagement='outside_verified',release='held_known',manual_latched=False)
    d.update(kw);return Context(**d)


def evidence(ctx=None,now=10.3):
    ctx=ctx or context();rows={}
    for i in range(1,21):
        name=f'G{i:02d}';details={}
        if name=='G07':details=dict(si_mapping_verified=True,braking_verified=True,odometry_control_verified=True)
        if name=='G08':details=dict(normal_clear=True,stopping_clear=True,coverage_complete=True)
        if name=='G12':details=dict(bound_kind='validated_end_to_end_bound',clearance_mm=1.2,error_upper_mm=.3,reserve_mm=.2,retreat_verified=True,contact_free=True,target_static=True,current_pose_known=True)
        if name=='G13':details=dict(actual_velocity_known=True,fresh_post_stop=True)
        if name=='G15':details=dict(support_verified=True,insertion_depth_verified=True)
        if name=='G16':details=dict(empty_grip_verified=True,release_verified=True)
        if name=='G17':details=dict(battery_left_in_slot=True,stable_verified=True)
        rows[name]=Evidence(name,'verified',now,now+2,ctx.run_id,ctx.boot_id,ctx.generation,ctx.asset_hash,details=details,state_seq=ctx.state_seq)
    return rows


def held():
    m=HeldBatteryMonitor(IDENTITY)
    for i,t in enumerate([10.,10.15,10.3]):r=m.update(observation(i,t),feedback(t),now=t,carry_certificate=certificate())
    return r


class GateTests(unittest.TestCase):
    def setUp(self):self.k=Gatekeeper();self.ctx=context();self.e=evidence(self.ctx);self.h=held()
    def run_gate(self,action,**kw):return self.k.evaluate(action,self.e,self.ctx,now=10.3,held=self.h,**kw)
    def test_verified_holding_is_wired_to_base_lift_and_arm_gate(self):
        for action in ('search','base_align','lift_align','outside_align','insert'):
            with self.subTest(action=action):self.assertTrue(self.run_gate(action).allowed)
        self.h['holding_eligibility']='unknown'
        for action in ('search','base_align','lift_align','outside_align','insert'):
            with self.subTest(action=action):self.assertIn('G09',self.run_gate(action).blocked)
    def test_gripper_alone_or_generic_green_G09_cannot_replace_held_monitor(self):
        self.assertIn('G09',self.k.evaluate('base_align',self.e,self.ctx,now=10.3,held=None).blocked)
    def test_no_target_does_not_block_safe_search_but_blocks_alignment(self):
        del self.e['G05'];self.assertTrue(self.run_gate('search').allowed);self.assertFalse(self.run_gate('base_align').allowed)
    def test_failed_movement_gates_do_not_block_diagnostic_or_stop(self):
        ctx=context(manual_latched=True)
        for action in ('inspect','stop'):
            self.assertTrue(self.k.evaluate(action,{},ctx,now=10.3).allowed)
    def test_nominal_pose_or_pixel_error_cannot_pass_insertion_precision(self):
        self.e['G12']=Evidence('G12','verified',10.3,12.,self.ctx.run_id,self.ctx.boot_id,1,self.ctx.asset_hash,
            details={'bound_kind':'reprojection_only','clearance_mm':1.2,'error_upper_mm':.01,'reserve_mm':.2})
        self.assertFalse(self.run_gate('insert').allowed);self.assertTrue(self.run_gate('outside_align').allowed)
    def test_precision_and_stopping_envelope_are_independent_hard_gates(self):
        for field in ('normal_clear','stopping_clear','coverage_complete'):
            e=evidence();e['G08'].details[field]=False
            self.assertIn('G08',self.k.evaluate('search',e,self.ctx,now=10.3,held=self.h).blocked)
        self.e['G12'].details['error_upper_mm']=1.
        self.assertFalse(self.run_gate('insert').allowed)
    def test_unknown_units_missing_evidence_nan_scope_or_stale_are_rejected(self):
        for mutation in ('missing','units','nan','stale','generation','real'):
            e=evidence()
            if mutation=='missing':del e['G07']
            elif mutation=='units':e['G07'].details['si_mapping_verified']=False
            else:
                old=e['G07'];d=old.__dict__.copy()
                if mutation=='nan':d['expires_at']=float('nan')
                elif mutation=='stale':d['expires_at']=10.
                elif mutation=='generation':d['generation']=0
                elif mutation=='real':d['scope']='real'
                e['G07']=Evidence(**d)
            self.assertIn('G07',self.k.evaluate('base_align',e,self.ctx,now=10.3,held=self.h).blocked)
    def test_partial_insertion_blocks_base_and_lift_even_if_target_returns(self):
        ctx=context(engagement='may_be_inserted')
        for action in ('search','base_align','lift_align'):
            r=self.k.evaluate(action,self.e,ctx,now=10.3,held=self.h);self.assertFalse(r.allowed)
    def test_independent_retreat_can_work_without_target_but_not_with_contact_unknown(self):
        ctx=context(engagement='may_be_inserted');del self.e['G05']
        self.assertTrue(self.k.evaluate('retreat',self.e,ctx,now=10.3,held=self.h).allowed)
        self.e['G12'].details['contact_free']=False
        self.assertFalse(self.k.evaluate('retreat',self.e,ctx,now=10.3,held=self.h).allowed)
    def test_manual_stop_is_latched_and_unknown_release_never_replayed(self):
        self.assertFalse(self.k.evaluate('search',self.e,context(manual_latched=True),now=10.3,held=self.h).allowed)
        for action in ('release','withdraw_empty','insert'):
            self.assertFalse(self.k.evaluate(action,self.e,context(release='unknown'),now=10.3,held=self.h).allowed)
    def test_release_and_empty_withdraw_use_different_evidence(self):
        self.assertFalse(self.run_gate('withdraw_empty').allowed)
        ctx=context(engagement='released_verified',release='released_verified')
        self.assertTrue(self.k.evaluate('withdraw_empty',self.e,ctx,now=10.3,held=None).allowed)
    def test_empty_withdraw_requires_explicit_empty_grip_evidence(self):
        ctx=context(engagement='released_verified',release='released_verified')
        self.e['G16'].details['empty_grip_verified']=False
        self.assertFalse(self.k.evaluate('withdraw_empty',self.e,ctx,now=10.3).allowed)
    def test_release_in_progress_cannot_reenter_insert_and_readonly_observation_survives_manual_stop(self):
        self.assertFalse(self.k.evaluate('insert',self.e,context(engagement='inside_verified',release='release_requested'),now=10.3,held=self.h).allowed)
        self.assertTrue(self.k.evaluate('observe',self.e,context(manual_latched=True),now=10.3).allowed)
    def test_visible_battery_still_in_grip_conflicts_with_empty_withdraw_certificate(self):
        ctx=context(engagement='released_verified',release='released_verified')
        self.assertFalse(self.k.evaluate('withdraw_empty',self.e,ctx,now=10.3,held=self.h).allowed)
    def test_lift_calibration_support_and_final_verification_have_independent_gates(self):
        for action,gate,ctx in [('lift_align','G10',self.ctx),('release','G15',context(engagement='inside_verified')),
                                ('complete','G17',context(engagement='disengaged_verified',release='released_verified'))]:
            rows=evidence(ctx);del rows[gate]
            self.assertIn(gate,self.k.evaluate(action,rows,ctx,now=10.3,held=self.h).blocked)
    def test_unregistered_global_navigation_or_torque_changes_are_not_actions(self):
        for action in ('navigate_global','set_torque','force_grip','clear_emergency_stop'):
            with self.assertRaises(ValueError):self.k.evaluate(action,self.e,self.ctx,now=10.3,held=self.h)
