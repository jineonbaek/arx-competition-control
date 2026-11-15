"""Real ControllerBridge task boundary; the offline simulator stays separate.

No browser-supplied trajectories/approval and no fabricated physical provider.
This task finishes at inserted hold; it cannot release, move lift or chassis.
"""
import copy
import json
from pathlib import Path
import threading
import uuid
import numpy as np
from agent.storage import atomic_json
from agent.cabinet_controller_bridge import ControllerBridge,BridgePolicy,digest

ROOT=Path(__file__).resolve().parents[2]

class CabinetExecutionTask:
    def __init__(self,controller,preview,*,directory=None,planner=None,physical_provider=None,
                 bridge_policy=None,auto_worker=True):
        self.ctrl=controller;self.preview=preview;self.directory=Path(directory or ROOT/'web/hardware/runtime/cabinet-execution')
        self.planner=planner or self._plan;self.provider=physical_provider
        self.policy=bridge_policy or BridgePolicy(.002,.02,.02,.02,.3,.2,.2,70.)
        self.auto_worker=auto_worker;self.lock=threading.RLock();self.tick_lock=threading.Lock()
        self.cancel=threading.Event();self.thread=None;self.bridge=None;self.bundle=None;self.index=0
        self.state=dict(status='idle',phase='idle',reason='',battery_placed=False,
                        outside_tolerance_mm=5.,physical_provider_configured=callable(physical_provider),
                        execution_enabled=callable(physical_provider),start_checks_available=True)
        try:
            old=json.loads((self.directory/'receipt.json').read_text())
            if type(old) is not dict:raise ValueError('Task receipt must be an object')
            if old.get('status') not in ('idle','blocked','cancelled','completed_hold'):
                self.state.update(status='recovery_required',reason='Previous task unfinished; inspect physical state, never replay')
        except FileNotFoundError:pass
        except (ValueError,TypeError,OSError):self.state.update(status='recovery_required',reason='Task receipt unreadable')
    def snapshot(self):
        with self.lock:return copy.deepcopy(self.state)
    def _save(self,**changes):
        with self.lock:
            self.state.update(changes);self.directory.mkdir(parents=True,exist_ok=True)
            atomic_json(self.directory/'receipt.json',self.state)
    def action(self,data):
        if data=={'op':'cancel'}:
            self.cancel.set()
            # Never hold the task lock while physical admission may be blocked.
            bridge=self.bridge
            if bridge is not None:bridge.cancel('operator cancel')
            with self.lock:
                if self.state['status'] not in ('idle','blocked','cancelled','completed_hold'):
                    self._save(status='cancelling',reason='Operator cancellation latched')
            return self.snapshot()
        if (type(data) is not dict or set(data)!={'op','model'} or data['op']!='start'
                or data['model'] not in ('demo','competition')):raise ValueError('Only current model start or cancel is accepted')
        with self.lock:
            if self.state['status'] not in ('idle','blocked','cancelled','completed_hold'):raise ValueError('Task active or recovery unresolved')
            if self.thread is not None and self.thread.is_alive():raise ValueError('Prior worker has not exited')
            s=self.ctrl.snapshot()
            if any(s.get(k,False) is not False for k in ('motion_reserved','competition_reserved','chassis_active')):raise ValueError('Robot control is busy')
            self.cancel=threading.Event();self.bridge=None;self.bundle=None;self.index=0
            self.initial=copy.deepcopy(s);self.expected_epoch=s['command_epoch'];self.config_hash=digest(self.ctrl.config)
            self._save(status='planning',phase='d435_scan',reason='',run_id=uuid.uuid4().hex,battery_placed=False)
            run_id=self.state['run_id']
        try:
            bundle=copy.deepcopy(self.planner(data['model']))
            if (not isinstance(bundle,dict) or not bundle.get('segments') or len(bundle['segments'])>64
                    or not isinstance(bundle.get('asset_hash'),str) or not isinstance(bundle.get('held_identity_hash'),str)):
                raise ValueError('No bound server-side segment plan')
            tcp=np.asarray(bundle['tcp_matrix'],float)
            from agent.placement_geometry import rigid
            rigid(tcp)
            for segment in bundle['segments']:
                if segment['phase'] not in ControllerBridge.PHASES:raise ValueError('Unsupported real segment phase')
                if segment['phase']=='outside_align':
                    xyz=np.asarray(segment['target_tcp_mm'],float)
                    if xyz.shape!=(3,) or not np.isfinite(xyz).all():raise ValueError('Outside TCP target unavailable')
            with self.lock:
                if self.cancel.is_set():self._save(status='cancelled');return self.snapshot()
                if self.state['run_id']!=run_id:raise ValueError('Planning generation changed')
                self.bundle=bundle;self._save(status='queued',phase=bundle['segments'][0]['phase'])
                if self.auto_worker:
                    self.thread=threading.Thread(target=self._run,name='cabinet-real-task',daemon=True);self.thread.start()
            return self.snapshot()
        except Exception as error:
            self._save(status='cancelled' if self.cancel.is_set() else 'blocked',reason=str(error)[:400])
            return self.snapshot()
    def _context(self,state):
        base=self.initial
        epochs={self.expected_epoch} if self.bridge is None or self.bridge.token is None else {self.expected_epoch+1,self.expected_epoch+2}
        if state['command_epoch'] not in epochs or digest(self.ctrl.config)!=self.config_hash:
            raise ValueError('Manual command or configuration changed after planning')
        values=[]
        for side in ('left','right'):
            row=state['arms'][side];old=base['arms'][side]
            if not row.get('fresh'):raise ValueError('Feedback expired after planning')
            values.append((row['gripper']-old['gripper'],.02))
            if side=='left':values.extend((a-b,.002) for a,b in zip(row['joints'],old['joints']))
        body=state['body'];old=base['body']
        if not body.get('fresh') or any(v!=0 for v in body['wheels']):raise ValueError('Body/chassis is not stationary')
        values.extend((body[k]-old[k],.02 if k=='height' else .002) for k in ('height','waist','head_yaw','head_pitch'))
        if any(not np.isfinite(v) or abs(v)>limit for v,limit in values):raise ValueError('Grip, lift, left arm or body changed after planning')
    def _approve(self,request,state,now):
        self._context(state)
        return self.provider(request,state,now)
    def tick(self):
        if not self.tick_lock.acquire(False):return self.snapshot()
        try:
            if self.state['status'] in ('idle','blocked','cancelled','completed_hold','recovery_required'):return self.snapshot()
            if self.cancel.is_set() and self.bridge is None:
                self._save(status='cancelled',reason='Cancelled before dispatch');return self.snapshot()
            segment=self.bundle['segments'][self.index]
            if self.bridge is None:
                self._context(self.ctrl.snapshot())
                bridge=ControllerBridge(self.ctrl,self.policy,physical_provider=self._approve if callable(self.provider) else None)
                with self.lock:self.bridge=bridge
                if self.cancel.is_set():bridge.cancel('operator cancel')
                binding=dict(run_id=self.state['run_id'],boot_id='web-'+str(id(self)),generation=self.index,
                             asset_hash=self.bundle['asset_hash'],held_identity_hash=self.bundle['held_identity_hash'])
                bridge.start(segment['phase'],segment['plan'],binding)
                self._save(status='running',phase=segment['phase'],segment=self.index)
            else:
                result=self.bridge.poll()
                if result['outcome_unknown'] or result['status']=='recovery_required':
                    self._save(status='recovery_required',reason=result['reason'])
                elif self.cancel.is_set() or result['cancellation_requested']:
                    self._save(status='cancelled' if result['status']=='cancelled' else 'cancelling')
                elif result['completed']:
                    # Existing Controller increments once each for reserve, dispatch and handoff.
                    # Any extra manual command invalidates the next segment, including a stop.
                    with self.ctrl.lock:
                        if self.ctrl.command_epoch!=self.expected_epoch+3:raise ValueError('Command epoch changed at handoff')
                        self.expected_epoch+=3
                        self._context(self.ctrl.snapshot())
                    if segment['phase']=='outside_align':
                        s=self.ctrl.snapshot();arm=s['arms']['right'];body=s['body']
                        if not arm['fresh'] or not body['fresh']:raise ValueError('Outside arrival feedback expired')
                        tool=self.ctrl.models['right'].wrist_transform(arm['joints'],body['height'])@np.asarray(self.bundle['tcp_matrix'])
                        error=float(np.linalg.norm(tool[:3,3]*1000-np.asarray(segment['target_tcp_mm'])))
                        if not np.isfinite(error) or error>5.+1e-9:
                            self._save(status='recovery_required',reason='Outside TCP arrival exceeds5mm; insertion not started',actual_tcp_error_mm=error);return self.snapshot()
                    with self.lock:
                        if self.cancel.is_set():self._save(status='cancelled');return self.snapshot()
                        self.index+=1
                        if self.index==len(self.bundle['segments']):self._save(status='completed_hold',reason='Right-arm segments settled; battery not released')
                        else:self.bridge=None;self._save(status='queued',phase=self.bundle['segments'][self.index]['phase'])
            return self.snapshot()
        except Exception as error:
            bridge=self.bridge
            if bridge is not None and bridge.snapshot()['ownership_retained']:
                bridge.cancel(str(error));self._save(status='recovery_required',reason=str(error)[:400])
            else:self._save(status='cancelled' if self.cancel.is_set() else 'blocked',reason=str(error)[:400])
            return self.snapshot()
        finally:self.tick_lock.release()
    def _run(self):
        while True:
            r=self.tick()
            if r['status'] in ('blocked','cancelled','completed_hold','recovery_required'):return
            # Cancellation must still allow bounded polling to verify actual rest.
            __import__('time').sleep(.02)
    def close(self):
        self.action({'op':'cancel'})
        if self.thread is not None:self.thread.join(2.)
    def _plan(self,model):
        result=self.preview.action({'camera':'d435','model':model},include_arm_preview=False)
        if result.get('status')!='localized' or not result.get('plan'):
            raise ValueError('D435 path rejected: '+result.get('reason',result.get('status','unknown')))
        if not result.get('validity',{}).get('valid'):raise ValueError('D435 plan is not current')
        if not callable(self.provider):raise ValueError('Validated physical provider missing; current geometry alone cannot approve motion')
        p=result['plan'];s=self.ctrl.snapshot();q=s['arms']['right']['joints'];lift=s['body']['height']
        model=self.ctrl.models['right'];limits=self.ctrl.config['arms']['right']['joint_limits_deg']
        tcp=np.asarray(json.loads((self.preview.root/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
        timing=dict(joint_speed_deg_s=[3.]*6,joint_acceleration_deg_s2=[6.]*6,tcp_speed_mm_s=12.,tcp_acceleration_mm_s2=25.)
        from agent.grasp_path import plan_correction,plan_insert
        target=np.eye(4);target[:3,:3]=p['R_base_from_tool'];target[:3,3]=np.asarray(p['tcp_pre_base_mm'])/1000
        outside=plan_correction(model,q,lift,tcp,target,limits,timing_limits=timing)
        insert=plan_insert(model,outside['joint_waypoints_rad'][-1],lift,p['tcp_pre_base_mm'],p['tcp_end_base_mm'],p['R_base_from_tool'],tcp,limits,timing_limits=timing)
        segments=[]
        for phase,path in [('outside_align',outside),('insert',insert)]:
            qs=path['joint_waypoints_rad'];times=path['timing']['time_from_start_s'];begin=0
            while begin<len(qs)-1:
                end=begin+1
                if times[end]-times[begin]>45:raise ValueError('Individual segment exceeds supervised duration')
                while end+1<len(qs) and times[end+1]-times[begin]<=45:end+=1
                plan=dict(interpolation='segment_quintic_scalar',time_from_start_s=[t-times[begin] for t in times[begin:end+1]],joint_waypoints_rad=qs[begin:end+1])
                xyz=(model.wrist_transform(qs[end],lift)@tcp)[:3,3]*1000
                if phase=='outside_align' and end==len(qs)-1:xyz=np.asarray(p['tcp_pre_base_mm'],float)
                segments.append(dict(phase=phase,plan=plan,target_tcp_mm=xyz.tolist()));begin=end
        return dict(segments=segments,tcp_matrix=tcp.tolist(),asset_hash=digest(result['source_hashes']),
                    held_identity_hash=digest(dict(gripper=s['arms']['right']['gripper'],tcp=tcp.tolist(),run=result['snapshot_id'])))
