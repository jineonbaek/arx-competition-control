"""Bind durable Agent requests to the existing local Controller steps."""
import copy
import hashlib
import json
import math
from threading import Lock
import time
import numpy as np
from scipy.spatial.transform import Rotation
from web.src.handeye_column import ColumnModel
from .battery_execution import prepare_step
from .coarse_alignment import observer_search_geometry
from .coarse_execution import CoarseAlignmentStep
from .grasp_path import TimedJointPath
from .pick_backend import CoarseExecutionBackend, ApproachExecutionBackend, PickExecutionBackend, ExtractionExecutionBackend
from .battery_extract import WristExtractionStep
from .storage import atomic_json

BACKENDS={c.operation:c for c in (CoarseExecutionBackend,ApproachExecutionBackend,PickExecutionBackend,ExtractionExecutionBackend)}

class BatteryControllerBackend:
    def __init__(self,controller,token,store,*,policies,pick_setup,preflight,cancelled,timeout_s):
        if store.closed or not callable(preflight) or not callable(cancelled):
            raise ValueError('Open run and local preflight/cancellation required')
        if set(timeout_s)!=set(BACKENDS)-{'stow_for_carry'} or any(type(t) not in (int,float) or not math.isfinite(t)
                                               or not 0<t<=3600 for t in timeout_s.values()):
            raise ValueError('Finite deadlines required for all execution stages')
        self.controller=controller;self.token=token;self.store=store
        self.policies=copy.deepcopy(policies);self.pick_setup=copy.deepcopy(pick_setup)
        self.preflight=preflight;self.cancelled=cancelled;self.timeout_s=dict(timeout_s)
        self.directory=store.path/'executions';self.directory.mkdir(exist_ok=True)
        self.entries={};self.lock=Lock();self.cancelled_here=False

    def available(self,operation):
        if operation=='verify_hold':
            return (callable(getattr(self.controller,'snapshot',None)) and not self.store.closed
                    and len(self.policies.get('pick',{}).get('hold_gripper_range',[]))==2)
        return (operation in BACKENDS and not self.store.closed and not self.cancelled_here
                and not self.store.cancelled and not self.cancelled()
                and self.controller.motion_valid(self.token)
                and (operation=='execute_coarse_alignment' or self.pick_setup is not None))

    def observer_search_offsets(self):
        return list(self.policies.get('coarse',{}).get('observer_search',{}).get('offsets_mm',[]))

    def adjust_observer_search(self,call_id,index,offset_mm,arm='right'):
        """Move only the observation arm along absolute configured Z offsets."""
        if arm!='right' or type(index) is not int or index<0 or not isinstance(call_id,str):
            raise ValueError('Invalid observer search identity')
        key=hashlib.sha256((self.store.state['run_id']+':'+call_id+':observer:'+str(index)).encode()).hexdigest()
        directory=self.directory/'observer-search';directory.mkdir(exist_ok=True)
        journal=directory/(key+'.json')
        if journal.exists():
            try:saved=json.loads(journal.read_text())
            except (OSError,ValueError) as error:raise ValueError('Observer search journal unreadable') from error
            if saved.get('status')=='completed':return copy.deepcopy(saved['data'])
            raise ValueError('Earlier observer search requires recovery')
        mission=self.store.state.get('mission') or {}
        plan=self.store.state.get('coarse_observer_plan') or mission.get('plan') or {}
        observer=plan.get('observer') or {}
        if mission.get('stage')!='COARSE_ALIGNED':
            raise ValueError('Observer search is only allowed after coarse alignment')
        if self.store.state.get('pending') is not None or not self.available('execute_coarse_alignment'):
            raise ValueError('Observer search cannot start while another execution is pending')
        p=copy.deepcopy(self.policies['coarse'])
        snapshot=self.controller.snapshot()
        q=np.asarray(snapshot['arms']['right']['joints'],float)
        lift=float(snapshot['body']['height'])
        if (snapshot['arms']['right'].get('fresh') is not True or snapshot['body'].get('fresh') is not True
                or q.shape!=(6,) or not np.isfinite(q).all() or not np.isfinite(lift) or not 0<=lift<=18):
            raise ValueError('Fresh right-arm and lift feedback required for observer search')
        model=ColumnModel()
        origin=self.store.state.get('observer_search_origin')
        if origin is None:
            if observer.get('status')!='fixed_observer_candidate':
                raise ValueError('固定观察姿态尚未通过检查')
            wrist=model.wrist_transform(q,lift)
            origin=dict(plan_id=plan.get('plan_id'),
                target_wrist_base_mm=(wrist[:3,3]*1000).tolist(),
                rpy_deg=Rotation.from_matrix(wrist[:3,:3]).as_euler('xyz',degrees=True).tolist())
            self.store.state['observer_search_origin']=origin
            self.store.save()
        if not isinstance(origin,dict) or origin.get('plan_id')!=plan.get('plan_id'):
            raise ValueError('Observer search origin belongs to another coarse plan')
        search_observer=observer|{k:origin[k] for k in ('target_wrist_base_mm','rpy_deg')}
        geometry=observer_search_geometry(search_observer,float(offset_mm),q,lift,model,
            self.controller.config['arms']['right']['joint_limits_deg'])
        if not geometry['clear']:raise ValueError('观察搜索路径接近中央立柱')
        target=np.asarray(geometry['joints_rad'],float)
        path=TimedJointPath([q,target],p['reach_bound_m'],
            tcp_position=lambda angles:model.wrist_transform(angles,lift)[:3,3],**p['timing_limits'])
        timed=dict(version=1,interpolation='segment_quintic_scalar',
            time_from_start_s=path.times.tolist(),joint_waypoints_rad=path.q.tolist())
        initial={side:np.asarray(snapshot['arms'][side]['joints'],float).copy() for side in ('left','right')}
        intent=dict(call_id=call_id,index=index,offset_mm=float(offset_mm),target=geometry['target_wrist_base_mm'])
        atomic_json(journal,dict(status='unknown',**intent))
        def guard(fresh,now):
            return self.preflight('execute_coarse_alignment',copy.deepcopy(plan),fresh,now) is True
        step=None
        try:
            step=CoarseAlignmentStep(self.controller,self.token,lift_target_rad=lift,
                lift_acceptance_rad=lift,
                observer_plan=timed,initial_joints=initial,initial_lift_rad=lift,
                fixed_joint_tolerance_rad=p['fixed_joint_tolerance_rad'],
                lift_policy=p['lift_policy'],arm_policy=p['arm_policy'],
                max_pair_skew_s=p['max_pair_skew_s'],lift_preflight=guard,arm_preflight=guard)
            receipt=step.start();deadline=time.monotonic()+self.timeout_s['execute_coarse_alignment']
            while receipt.get('status')!='ready_to_observe' and not receipt.get('stop_required'):
                if self.cancelled():receipt=step.cancel()
                elif time.monotonic()>=deadline:receipt=step.timeout()
                else:time.sleep(.02);receipt=step.poll(cancelled=self.cancelled())
            if receipt.get('status')!='ready_to_observe':
                raise ValueError('Observer search failed: '+str(receipt.get('status')))
            final=self.controller.snapshot()
            data=dict(status='completed',offset_mm=float(offset_mm),
                target_wrist_base_mm=geometry['target_wrist_base_mm'],
                joints_rad=np.asarray(final['arms']['right']['joints'],float).tolist(),
                lift_rad=float(final['body']['height']))
            atomic_json(journal,dict(status='completed',**intent,data=data))
            return data
        except Exception as error:
            if step is not None:
                try:step.cancel()
                except Exception:pass
            atomic_json(journal,dict(status='failed',**intent,error=str(error)[:180]))
            raise ValueError('Observer search failed: '+str(error)[:180]) from error

    def _path(self,request):
        if not isinstance(request,str) or len(request)>256 or not request.startswith(self.store.state['run_id']+':'):
            raise ValueError('Request belongs to another run')
        return self.directory/(hashlib.sha256(request.encode()).hexdigest()+'.json')

    @staticmethod
    def _reply(status,request,reason):
        return dict(status=status,execution_id=request,data=dict(reason=reason,grasp_confirmed=False))

    def _stored(self,path,operation,request,payload=None):
        try:
            saved=json.loads(path.read_text());receipt=saved['receipt']
            if saved['operation']!=operation or receipt['execution_id']!=request:
                raise ValueError('Journal identity mismatch')
            if payload is not None and saved['payload']!=payload:
                return self._reply('rejected',request,'bound_plan_mismatch')
            if receipt['status'] in ('completed','rejected'):return receipt
        except (OSError,ValueError,KeyError,TypeError):pass
        return self._reply('unknown',request,'recovery_required')

    def invoke(self,operation,arguments,request_id):
        if operation=='verify_hold':
            try:
                self._path(request_id)
                if not self.available(operation):raise ValueError('Hold verifier unavailable')
                arm=self.controller.snapshot().get('arms',{}).get('right',{})
                feedback=arm.get('gripper');low,high=self.policies['pick']['hold_gripper_range']
                if (arm.get('fresh') is not True or type(feedback) not in (int,float)
                        or not math.isfinite(feedback)):
                    raise ValueError('Fresh right gripper feedback required')
                if not low<=feedback<=high:
                    raise ValueError('Gripper feedback outside configured hold range')
                evidence=dict(hold_status='confirmed',target_id=(arguments.get('target') or {}).get('target_id'),
                    gripper_feedback_in_range=True,gripper_feedback=float(feedback),
                    gripper_feedback_range=[float(low),float(high)])
                return dict(status='completed',execution_id=request_id,data=evidence)
            except (OSError,ValueError,KeyError,TypeError) as error:
                return self._reply('rejected',request_id,str(error)[:180])
        with self.lock:
            try:
                path=self._path(request_id)
                payload=json.dumps(arguments,sort_keys=True,allow_nan=False)
                if operation not in BACKENDS:raise ValueError('Unknown execution stage')
                if request_id in self.entries:
                    return self.entries[request_id].invoke(operation,arguments,request_id)
                if path.exists():return self._stored(path,operation,request_id,payload)
                if not self.available(operation):raise ValueError('Execution capability unavailable')
                pending=self.store.state.get('pending') or {}
                if (pending.get('request_id')!=request_id or pending.get('operation')!=operation
                        or pending.get('arguments')!=arguments):raise ValueError('Pending request mismatch')
                if arguments.get('plan')!=self.store.state['mission'].get('plan'):
                    raise ValueError('Mission plan mismatch')
                # An unverified earlier stop cannot be made safe by choosing a new request ID.
                for journal in self.directory.glob('*.json'):
                    saved=json.loads(journal.read_text())
                    if saved['receipt']['status'] not in ('completed','rejected'):
                        raise ValueError('Earlier execution requires recovery')
                if operation=='stow_for_carry':
                    step=WristExtractionStep(self.controller,self.token,refinement=self.policies['refinement'])
                else:
                    step=prepare_step(operation,arguments['plan'],self.controller,self.token,
                        policies=self.policies,pick_setup=self.pick_setup,preflight=self.preflight)
                entry=BACKENDS[operation](step,request_id=request_id,arguments=arguments,journal=path,
                    cancelled=lambda:self.store.closed or self.store.cancelled or self.cancelled(),
                    timeout_s=self.timeout_s.get(operation,self.timeout_s['execute_pick']))
                self.entries[request_id]=entry
                return entry.invoke(operation,arguments,request_id)
            except (OSError,ValueError,KeyError,TypeError) as error:
                # A journal may already exist if startup failed after durable intent was written.
                if 'path' in locals() and path.exists():
                    return self._reply('unknown',request_id,'recovery_required: '+str(error)[:160])
                return self._reply('rejected',request_id,str(error)[:180])

    def status(self,operation,request_id):
        with self.lock:
            if request_id in self.entries:return self.entries[request_id].status(operation,request_id)
            try:return self._stored(self._path(request_id),operation,request_id)
            except ValueError:return self._reply('unknown',request_id,'unknown_request')

    def cancel_and_wait(self,timeout):
        with self.lock:
            self.cancelled_here=True
            entries=list(self.entries.values())
        deadline=time.monotonic()+max(0,timeout)
        results=[entry.cancel_and_wait(max(0,deadline-time.monotonic())) for entry in entries]
        return all(results)
