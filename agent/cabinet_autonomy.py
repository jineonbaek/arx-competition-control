"""Durable offline cabinet flow. Uses only file I/O and the in-memory simulator."""
from dataclasses import replace
import copy
import fcntl
import json
import math
from pathlib import Path
import threading
import numpy as np
from .storage import atomic_json
from .held_battery import HeldBatteryMonitor,finite
from .cabinet_gates import Context,Evidence,Gatekeeper,identity_hash
from .cabinet_docking import DockingPlanner,SearchBudget
from .cabinet_simulator import SimulationExecutor


class CabinetTracker:
    def __init__(self):self.rows=[];self.seen={};self.target_id=None;self.last_stamp=-math.inf
    def invalidate_window(self):self.rows=[]
    def update(self,row,*,now):
        try:
            stamp=row['captured_at'];point=np.asarray(row['center_local_m'],float);normal=np.asarray(row['normal_local'],float)
            if (row['camera']!='d435' or row['scope']!='simulation' or row['frame']!='local_gravity' or row.get('ambiguous') is not False
                    or not finite(now) or not finite(stamp) or not 0<=now-stamp<=2. or stamp<self.last_stamp
                    or point.shape!=(3,) or normal.shape!=(3,) or not np.isfinite(point).all() or not np.isfinite(normal).all()
                    or abs(np.linalg.norm(normal)-1)>.01 or not row.get('target_id') or not row.get('frame_id')):
                raise ValueError('Invalid cabinet identity, exposure, coordinates or ambiguity')
            key=(row['target_id'],row['model_sha256'])
            if self.target_id is not None and key!=self.target_id:raise ValueError('Target identity changed')
            self.target_id=key;encoded=json.dumps(row,sort_keys=True,allow_nan=False)
            if row['frame_id'] in self.seen:
                if self.seen[row['frame_id']]!=encoded:raise ValueError('Conflicting cached target')
                return self.snapshot(now=now)
            if stamp<=self.last_stamp:raise ValueError('Non-increasing exposure')
            self.last_stamp=stamp;self.seen[row['frame_id']]=encoded
            if self.rows and (np.linalg.norm(point-self.rows[-1]['center_local_m'])>.02 or normal@self.rows[-1]['normal_local']<math.cos(math.radians(5))):
                self.rows=[]
            self.rows.append(copy.deepcopy(row));self.rows=[r for r in self.rows if stamp-r['captured_at']<=1.]
            return self.snapshot(now=now)
        except (KeyError,ValueError,TypeError):self.rows=[];return dict(ready=False,reason='Cabinet observation invalid/changed/ambiguous',mode='offline',executable=False)
    def snapshot(self,*,now):
        ready=(finite(now) and len(self.rows)>=3 and self.rows[-1]['captured_at']-self.rows[0]['captured_at']>=.2
               and 0<=now-self.rows[-1]['captured_at']<=2.)
        return dict(ready=bool(ready),latest=copy.deepcopy(self.rows[-1]) if self.rows else None,
                    reason='Stable distinct D435 observations' if ready else 'Waiting for independent fresh D435 observations',mode='offline',executable=False)


class OfflineCabinetRun:
    def __init__(self,directory,context,held_identity,*,now,executor=None,parent_owner=None,initial_state=None):
        if not isinstance(context,Context) or not finite(now):raise ValueError('Offline context and finite clock required')
        if executor is not None and type(executor) is not SimulationExecutor:raise ValueError('Only the in-memory simulator is supported')
        if identity_hash(held_identity)!=context.held_identity_hash:raise ValueError('Held identity does not match context')
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True);self.lock=threading.RLock();self.closed=False
        self.handle=(self.directory/'writer.lock').open('a+');self.borrowed=parent_owner is not None;self.owner=None
        try:fcntl.flock(self.handle.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError:self.handle.close();raise ValueError('An offline run writer already owns this directory') from None
        try:
            self.base_context=context;self.executor=executor or SimulationExecutor(context.run_id,context.boot_id)
            if (self.executor.run_id,self.executor.boot_id)!=(context.run_id,context.boot_id):raise ValueError('Parent simulator identity mismatch')
            self.monitor=HeldBatteryMonitor(held_identity);self.held=None;self.tracker=CabinetTracker();self.gates=Gatekeeper();self.planner=DockingPlanner();self.budget=SearchBudget();self.last_now=now
            path=self.directory/'state.json'
            if path.exists():
                self.state=json.loads(path.read_text())
                if self.state.get('run_id')!=context.run_id:raise ValueError('Existing run identity mismatch')
                self.state.update(status='recovery_required',manual_latched=True,reason='Restart: inspect unresolved physical/stop state; never auto-replay')
                self.executor.generation=max(self.state.get('generation',0),self.executor.generation)+1
                self.executor.state_seq=self.state.get('state_seq',0)
                if self.state.get('inflight') or self.state.get('release')=='unknown':self.executor.outcome_unknown=True
                saved=self.state.get('budget',{})
                for attr,key in [('xy','xy_m'),('yaw','yaw_rad'),('distance','total_distance_m'),('rotation','total_rotation_rad'),('views','views')]:
                    if key in saved:setattr(self.budget,attr,np.array(saved[key]) if attr=='xy' else saved[key])
            else:
                self.owner=(self.executor.borrow(parent_owner,'cabinet-place') if self.borrowed else self.executor.acquire('cabinet-standalone',now=now))
                self.state=dict(schema_version=1,run_id=context.run_id,boot_id=context.boot_id,asset_hash=context.asset_hash,
                    phase='observe',status='ready',engagement=context.engagement,release=context.release,manual_latched=context.manual_latched,
                    inflight=None,records=[],selected=None,actual_state=copy.deepcopy(initial_state),correction_count=0,no_progress=0,last_error=None,correction_exhausted=False,reason='')
                if self._actual_valid(initial_state):
                    self.budget.xy=np.array(initial_state['base_xy_m'],float);self.budget.yaw=initial_state['base_yaw_rad']
            self._save()
        except Exception:
            fcntl.flock(self.handle.fileno(),fcntl.LOCK_UN);self.handle.close();self.closed=True;raise

    @property
    def context(self):
        return replace(self.base_context,generation=self.executor.generation,state_seq=self.executor.state_seq,
            engagement=self.state['engagement'],release=self.state['release'],manual_latched=self.state['manual_latched'] or self.executor.manual_latched)

    def _clock(self,now):
        if self.closed or not finite(now) or now<self.last_now:raise ValueError('Closed run or invalid/backward clock')
        self.last_now=now

    @staticmethod
    def _actual_valid(row):
        try:
            xy=np.asarray(row['base_xy_m'],float)
            return (row.get('scope')=='simulation' and xy.shape==(2,) and np.isfinite(xy).all() and np.linalg.norm(xy)<=.30
                    and finite(row['lift_rad']) and 0<=row['lift_rad']<=18 and finite(row['base_yaw_rad']) and abs(row['base_yaw_rad'])<=math.radians(15))
        except (KeyError,TypeError,ValueError):return False

    def _save(self):
        self.state.update(generation=self.executor.generation,state_seq=self.executor.state_seq,budget=self.budget.snapshot())
        atomic_json(self.directory/'state.json',self.state)

    def snapshot(self):
        with self.lock:return copy.deepcopy(self.state)|dict(mode='offline',executable=False)

    def observe_target(self,row,*,now):
        with self.lock:
            self._clock(now);result=self.tracker.update(row,now=now)
            pending=self.state.get('inflight')
            if pending and pending['action'] in ('base_align','outside_align','insert') and not result['ready'] and not self.executor.stopping:
                self.pause(now=now,reason='target_evidence_lost')
            return result

    def observe_held(self,row,feedback,*,now,carry_certificate=None):
        with self.lock:
            self._clock(now);phase=self.state['release'] if self.state['release'] in ('release_requested','released_verified') else 'carrying'
            self.held=self.monitor.update(row,feedback,now=now,carry_certificate=carry_certificate,phase=phase)
            pending=self.state.get('inflight');carrying=phase=='carrying'
            if carrying and (self.held['stability']=='slip_suspected' or self.held['holding_eligibility']=='blocked'
                    or pending and pending['action'] not in ('withdraw_empty',) and self.held['holding_eligibility']!='eligible'):
                self.pause(now=now,reason='holding_fault')
            return copy.deepcopy(self.held)

    def note_possible_insertion(self):
        with self.lock:
            if self.state['engagement'] in ('outside_verified','disengaged_verified'):self.state['engagement']='may_be_inserted';self._save()

    def request_correction(self,error_mm):
        with self.lock:
            if not finite(error_mm) or error_mm<0 or self.state['engagement'] not in ('outside_verified','disengaged_verified') or self.state['correction_exhausted']:return False
            old=self.state['last_error'];self.state['correction_count']+=1
            if old is not None and error_mm>=old*.99:self.state['no_progress']+=1
            else:self.state['no_progress']=0
            self.state['last_error']=error_mm
            permitted=self.state['correction_count']<=3 and self.state['no_progress']<2
            if not permitted:self.state.update(correction_exhausted=True,status='hold',reason='Correction budget exhausted or two cycles without progress')
            self._save();return permitted

    def step(self,evidence,*,now,check_candidate=None):
        with self.lock:
            self._clock(now)
            def reply(action='inspect',decision=None,segment=None):
                return dict(mode='offline',executable=False,next_action=action,decision=None if decision is None else decision.as_dict(),
                            segment=segment,state=self.snapshot(),held=copy.deepcopy(self.held),target=self.tracker.snapshot(now=now))
            pending=self.state.get('inflight')
            if pending and not self.executor.stopping:
                self.executor.tick(now=now)
                if not self.executor.stopping:
                    target=self.tracker.snapshot(now=now);rows=dict(evidence)
                    if not target['ready']:rows.pop('G05',None)
                    progress='release_progress' if pending['action']=='release' else pending['action']
                    decision=self.gates.evaluate(progress,rows,self.context,now=now,held=self.held)
                    if not decision.allowed:self.pause(now=now,reason='inflight_evidence_invalid')
                    else:
                        try:self.executor.renew(pending['segment_id'],self.owner,decision,now=now)
                        except ValueError:self.pause(now=now,reason='inflight_lease_invalid')
                if self.executor.stopping:self.state.update(status='stopping',reason=self.state.get('reason') or 'In-flight evidence expired');self._save()
            if self.executor.stopping and self.state['status'] not in ('completed','completed_pending_stop','recovery_required'):
                self.state.update(status='stopping',manual_latched=self.state['manual_latched'] or self.executor.manual_latched,
                    reason=self.state.get('reason') or 'Parent/shared simulation controller requested stop');self._save()
            if (self.state['status'] in ('stopping','recovery_required','completed','completed_pending_stop') or self.state['manual_latched']
                    or self.executor.stopping or self.state['inflight'] or self.owner is None or self.state['correction_exhausted']):return reply()
            target=self.tracker.snapshot(now=now);phase=self.state['phase']
            if phase.startswith('reobserve_'):
                if not target['ready']:return reply()
                self.state['phase']='lift_align' if phase=='reobserve_after_base' else ('outside_align' if phase=='reobserve_after_lift' else 'insert_gate')
                phase=self.state['phase']
            if phase=='observe':action='base_align' if target['ready'] else 'search'
            else:action={'lift_align':'lift_align','outside_align':'outside_align','insert_gate':'insert','support_check':'release','withdraw':'withdraw_empty','verify':'complete'}.get(phase)
            if action is None:return reply()
            rows=dict(evidence)
            # G05 is produced from actual tracking, never a caller's generic green flag.
            rows['G05']=Evidence('G05','verified' if target['ready'] else 'missing',now,now+min(2.,max(0.,2.-(now-target['latest']['captured_at'])) if target['latest'] else 0.),
                self.context.run_id,self.context.boot_id,self.context.generation,self.context.asset_hash,state_seq=self.context.state_seq)
            decision=self.gates.evaluate(action,rows,self.context,now=now,held=self.held)
            if not decision.allowed:
                d=rows.get('G12');details=d.details if isinstance(d,Evidence) else {}
                outside=self.gates.evaluate('outside_align',rows,self.context,now=now,held=self.held)
                if (action=='insert' and 'G12' in decision.blocked and outside.allowed and isinstance(details,dict)
                        and details.get('bound_kind')=='validated_end_to_end_bound' and details.get('correction_safe_outside') is True
                        and self.request_correction(details.get('correction_error_mm'))):
                    self.state.update(phase='outside_align',status='ready',reason='Bounded outside correction requested; insertion still blocked')
                else:self.state.update(status='hold',reason='; '.join(f'{k}: {v}' for k,v in decision.blocked.items()))
                self._save();return reply(decision=decision)
            if action!='complete' and not self._actual_valid(self.state.get('actual_state')):
                self.state.update(status='hold',reason='Actual simulation pose/lift missing, invalid or outside local region');self._save();return reply(decision=decision)
            if action=='complete':
                self.state.update(status='completed_pending_stop',reason='Completion evidence accepted; waiting for verified stop')
                self.executor.request_stop('completed',now=now);self._save();return reply(action,decision)
            if action=='base_align':
                proposal=self.planner.propose(target['latest']['center_local_m'],current_lift_rad=self.state['actual_state']['lift_rad'],check=check_candidate)
                if proposal['selected'] is None:self.state.update(status='hold',reason='No certified offline docking candidate');self._save();return reply(decision=decision)
                self.state['selected']=proposal['selected']
            try:
                if action in ('base_align','search'):
                    if action=='base_align':point=self.state['selected']['base_delta_m'];yaw=self.state['selected']['yaw_rad']
                    else:
                        sequence=[([0.,0.],-.1),([0.,0.],.1),([.05,0.],0.),([-.05,0.],0.)]
                        point,yaw=sequence[self.budget.views%len(sequence)]
                    self.budget.commit(point,yaw)
                if action=='insert':self.note_possible_insertion()
                if action=='release':self.state['release']='release_requested'
                segment_id=f'{self.context.run_id}:segment-{len(self.state["records"])+1}'
                goal={}
                if action in ('base_align','search'):goal=dict(base_xy_m=list(point),base_yaw_rad=yaw)
                elif action=='lift_align':goal=dict(lift_rad=self.state['selected']['lift_rad'])
                self.state.update(inflight={'segment_id':segment_id,'action':action,'state_seq':self.context.state_seq,'issued_at':now,'goal':goal},status='running',reason='')
                self._save()  # durable before dispatch; a crash here remains an unresolved outcome
                segment=self.executor.submit(segment_id,self.owner,decision,now=now)
                return reply(action,decision,segment)
            except ValueError as error:
                self.state.update(status='recovery_required' if self.state['inflight'] else 'hold',reason=str(error));self._save();return reply(decision=decision)

    def complete_segment(self,segment_id,*,now,completion):
        with self.lock:
            self._clock(now);pending=self.state.get('inflight')
            if not pending or segment_id!=pending['segment_id']:raise ValueError('No matching in-flight segment')
            c=completion
            valid=(isinstance(c,dict) and c.get('scope')=='simulation' and c.get('segment_id')==segment_id
                and c.get('actual_pose_verified') is True and c.get('rest_verified') is True
                and type(c.get('actual_state_seq')) is int and c['actual_state_seq']>pending['state_seq']
                and finite(c.get('completed_at')) and pending['issued_at']<=c['completed_at']<=now and now-c['completed_at']<=.5)
            if valid:
                actual=c.get('actual_state');valid=self._actual_valid(actual)
                if valid:
                    for key,value in pending.get('goal',{}).items():
                        limit=.05 if key=='lift_rad' else .005
                        if np.max(abs(np.asarray(actual[key])-value))>limit:valid=False
            if not valid:
                self.executor.receipt(segment_id,'unknown',now=now)
                if pending['action']=='release':self.state['release']='unknown'
                self.state.update(status='recovery_required',reason='Segment outcome unknown; no retry or release replay');self._save();return self.snapshot()
            self.executor.receipt(segment_id,'completed',now=now)
            try:self.budget.reconcile(c['actual_state']['base_xy_m'],c['actual_state']['base_yaw_rad'])
            except ValueError:
                self.executor.request_stop('measured_budget_exceeded',now=now);self.state.update(status='recovery_required',reason='Measured movement exceeded local budget');self._save();return self.snapshot()
            self.executor.state_seq=c['actual_state_seq'];self.state['actual_state']=copy.deepcopy(c['actual_state'])
            self.state['records'].append(pending|{'completion':copy.deepcopy(c),'status':'completed'});self.state['inflight']=None
            action=pending['action']
            next_phase={'search':'observe','base_align':'reobserve_after_base','lift_align':'reobserve_after_lift',
                        'outside_align':'reobserve_after_outside','insert':'support_check','release':'withdraw','withdraw_empty':'verify'}[action]
            if action=='insert':self.state['engagement']='inside_verified'
            if action=='release':self.state.update(release='released_verified',engagement='released_verified')
            if action=='withdraw_empty':self.state['engagement']='disengaged_verified'
            if self.executor.stopping or self.state['manual_latched']:
                self.state.update(phase=next_phase,status='stopping',reason='Late completion recorded; manual/stop latch remains');self._save();return self.snapshot()
            self.state.update(phase=next_phase,status='ready');self.tracker.invalidate_window();self.monitor.history=[];self.held=None;self._save();return self.snapshot()

    def pause(self,*,now,reason='manual'):
        with self.lock:
            self._clock(now)
            if reason in ('manual','takeover','cancel','operator_lease_lost'):self.state['manual_latched']=True
            self.state.update(status='stopping',reason=reason);self._save()
            result=self.executor.request_stop(reason,now=now);self._save();return result

    def observe_rest(self,row,*,now):
        with self.lock:
            self._clock(now);result=self.executor.observe_rest(row,now=now)
            if result['stopped_verified'] and not self.state['inflight'] and not self.executor.outcome_unknown:
                if not self.borrowed and self.owner is not None:self.executor.release(self.owner);self.owner=None
                self.state['status']='completed' if self.state['status']=='completed_pending_stop' else 'paused'
            self._save();return result

    def resume(self,*,expected_generation,actual_state,now):
        with self.lock:
            self._clock(now)
            if (type(expected_generation) is not int or expected_generation!=self.executor.generation or self.borrowed
                    or self.state['status']!='paused' or self.owner is not None or self.executor.owner is not None
                    or not self.executor.rest_verified or self.executor.outcome_unknown or self.state['inflight']
                    or self.state['engagement'] not in ('outside_verified','disengaged_verified') or self.state['release']!='held_known'
                    or self.state['correction_exhausted'] or not self._actual_valid(actual_state)):
                raise ValueError('Resume requires a resolved verified stop, known outside pose and explicit fresh generation')
            if (actual_state.get('measured_rest_verified') is not True or not finite(actual_state.get('captured_at'))
                    or not 0<=now-actual_state['captured_at']<=.5 or type(actual_state.get('actual_state_seq')) is not int
                    or actual_state['actual_state_seq']<=self.executor.state_seq):
                raise ValueError('Fresh measured actual-state evidence required for resume')
            distance=actual_state.get('unobserved_distance_upper_m');rotation=actual_state.get('unobserved_rotation_upper_rad')
            if not finite(distance) or not finite(rotation):raise ValueError('Bounded actual travel since pause is required; budget cannot reset')
            self.budget.reconcile(actual_state['base_xy_m'],actual_state['base_yaw_rad'],distance_upper_m=distance,rotation_upper_rad=rotation)
            self.owner=self.executor.acquire('cabinet-explicit-resume',now=now);self.executor.manual_latched=False
            self.executor.state_seq=actual_state['actual_state_seq'];self.state.update(actual_state=copy.deepcopy(actual_state),status='ready',manual_latched=False,reason='Explicit resume; all observations and gates must be rebuilt')
            self.tracker.invalidate_window();self.monitor.history=[];self.held=None;self._save();return self.snapshot()

    def close(self):
        with self.lock:
            if self.closed:return
            if self.state.get('inflight'):
                self.state.update(status='recovery_required',reason='Closing with unresolved segment; never auto-replay')
                if self.state['inflight']['action']=='release':self.state['release']='unknown'
                self._save()
            fcntl.flock(self.handle.fileno(),fcntl.LOCK_UN);self.handle.close();self.closed=True
