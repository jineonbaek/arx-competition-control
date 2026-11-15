"""One-shot supervised empty-gripper rehearsal. Absent local approval means no motion."""
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import numpy as np
from agent.storage import atomic_json

ROOT=Path(__file__).resolve().parents[2]
def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def trajectory_digest(stages):return hashlib.sha256(json.dumps(stages,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()

def default_sources(directory):
    from handeye_column import PACKAGE
    import xml.etree.ElementTree as ET
    paths={'tcp':ROOT/'config/right-grip-tcp.json','scene':directory/'frame.npz',
        'control':ROOT/'web/config/control.json','profiles':ROOT/'config/cabinet-profiles.json',
        'urdf':PACKAGE/'urdf/lift2_with_sensors.urdf',
        'calibration':ROOT/'vision/handeye/results/d435-cad-349622074226.json',
        'cabinet':ROOT/'vision/models/cabinet/demo.stl'}
    xml=ET.parse(paths['urdf'])
    for name in ('base_link','body4'):
        mesh=xml.find("link[@name='%s']/visual/geometry/mesh"%name)
        paths[name]=PACKAGE/mesh.get('filename').removeprefix('package://lift2/')
    for side,first in [('left',11),('right',21)]:
        for n in range(first,first+8):
            name=side+'_link'+str(n);mesh=xml.find("link[@name='%s']/visual/geometry/mesh"%name)
            paths[name]=PACKAGE/mesh.get('filename').removeprefix('package://lift2/')
    return paths

def validate_bundle(b,now):
    try:
        if (b['mode']!='empty_30mm' or not isinstance(b['run_id'],str) or not 1<=len(b['run_id'])<=80
            or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in b['run_id'])
            or not all(type(b[k]) in (int,float) and math.isfinite(b[k]) for k in ['created_at','expires_at'])
            or not b['created_at']<=now<b['expires_at']<=b['created_at']+180
            or not 0<b['insertion_depth_mm']<=30):raise ValueError('空载计划无效或已过期')
        from agent.grasp_geometry import rigid
        rigid(b['tcp_matrix'])
        for arm in b['initial']['arms'].values():
            if not np.isfinite(np.r_[arm['joints'],arm['gripper']]).all():raise ValueError('冻结关节反馈非有限数值')
        if not np.isfinite([b['initial']['body'][k] for k in ['height','waist','head_yaw','head_pitch']]).all():raise ValueError('冻结机体反馈非有限数值')
        cert=b['certificate']
        if (cert['continuous_sweep_checked'] is not True or cert['nominal_closed_jaw_only'] is not True
            or not math.isfinite(cert['continuous_lower_mm']) or cert['continuous_lower_mm']<=10
            or cert['trajectory_sha256']!=trajectory_digest(b['stages'])):raise ValueError('路径与连续扫掠证明不匹配')
        if not {'tcp','scene'}<=b['source_hashes'].keys() or any(not isinstance(v,str) or len(v)!=64 or any(c not in '0123456789abcdef' for c in v) for v in b['source_hashes'].values()):raise ValueError('缺少固定现场与TCP来源')
        previous=None;total=0
        if not 3<=len(b['stages'])<=8 or [s['name'] for s in b['stages'][-2:]]!=['insert','retract']:raise ValueError('必须仓外对齐、浅入、原路撤回')
        for stage in b['stages']:
            plan=stage['plan'];q=np.asarray(plan['joint_waypoints_rad'],float);t=np.asarray(plan['time_from_start_s'],float)
            if (set(plan)!={'interpolation','joint_waypoints_rad','time_from_start_s'} or plan['interpolation']!='segment_quintic_scalar'
                or q.ndim!=2 or q.shape[1]!=6 or not 2<=len(q)<=1024 or t.shape!=(len(q),)
                or not np.isfinite(q).all() or not np.isfinite(t).all() or t[0]!=0 or np.any(np.diff(t)<=0) or t[-1]>60):raise ValueError('轨迹格式或时长无效')
            dq=abs(np.diff(q,axis=0));dt=np.diff(t)[:,None]
            if np.any(np.degrees(dq)*1.875/dt>3.00001) or np.any(np.degrees(dq)*5.773503/dt**2>6.00001):raise ValueError('轨迹超过本次低速限值')
            if previous is not None and not np.allclose(previous,q[0],atol=1e-9,rtol=0):raise ValueError('分段轨迹不连续')
            previous=q[-1];total+=float(t[-1])
        if total>210:raise ValueError('本次空载路线过长')
        if not np.allclose(b['stages'][-1]['plan']['joint_waypoints_rad'],np.asarray(b['stages'][-2]['plan']['joint_waypoints_rad'])[::-1],atol=1e-9,rtol=0):raise ValueError('撤回必须沿原关节路径反向')
        return copy.deepcopy(b)
    except (KeyError,TypeError,OverflowError,IndexError) as e:raise ValueError('空载计划数据不完整') from e

class CabinetRehearsal:
    def __init__(self,controller,directory=None,*,source_paths=None,poll_s=.02,arrival_samples=4,arrival_interval_s=.12,arrival_timeout_s=2.5):
        self.ctrl=controller;self.directory=Path(directory or ROOT/'web/hardware/runtime/empty-rehearsal');self.sources=source_paths if source_paths is not None else default_sources(self.directory)
        self.poll_s=poll_s;self.arrival_samples=arrival_samples;self.arrival_interval_s=arrival_interval_s;self.arrival_timeout_s=arrival_timeout_s;self.last_source_check=0.
        self.lock=threading.RLock();self.cancel=threading.Event();self.done=threading.Event();self.thread=None;self.token=None;self.reference=None;self.bundle=None;self.source_identity=None;self.state=dict(status='disabled',phase='idle',battery_placed=False,reason='未安装本次固定空载路线的现场许可')
    def snapshot(self,*_):
        with self.lock:return copy.deepcopy(self.state)
    def _save(self,**data):
        with self.lock:
            self.state.update(data);self.directory.mkdir(parents=True,exist_ok=True);atomic_json(self.directory/'receipt.json',self.state)
    def _sources_valid(self,b):
        return set(self.sources)==set(b['source_hashes']) and all(Path(p).is_file() and digest(p)==b['source_hashes'][k] for k,p in self.sources.items())
    def _load(self,run_id):
        try:
            raw=(self.directory/'bundle.json').read_bytes();b=validate_bundle(json.loads(raw),time.time());a=json.loads((self.directory/'approval.json').read_bytes())
            if (run_id!=b['run_id'] or a['run_id']!=run_id or a['bundle_sha256']!=hashlib.sha256(raw).hexdigest()
                or type(a['expires_at']) not in (int,float) or not math.isfinite(a['expires_at'])
                or not time.time()<a['expires_at']<=b['expires_at'] or not self._sources_valid(b)):raise ValueError('计划、来源或现场许可已变化')
            tcp_bytes=Path(self.sources['tcp']).read_bytes()
            if hashlib.sha256(tcp_bytes).hexdigest()!=b['source_hashes']['tcp']:raise ValueError('工具文件在读取时发生变化')
            tcp=json.loads(tcp_bytes)['T_wrist_from_tool']
            if not np.allclose(tcp,b['tcp_matrix'],atol=1e-12,rtol=0):raise ValueError('工具变换与本机标定不一致')
            self.source_identity={k:(Path(p).stat().st_ino,Path(p).stat().st_size,Path(p).stat().st_mtime_ns) for k,p in self.sources.items()}
            self.last_source_check=time.monotonic()
            return b
        except (OSError,KeyError,TypeError,json.JSONDecodeError) as e:raise ValueError('没有本次空载路线的有效现场许可') from e
    def _guard(self,s,_,*,starting=False):
        try:
            initial=self.bundle['initial'];body=s['body'];ref=initial['body']
            if self.cancel.is_set():return False
            if starting or time.monotonic()-self.last_source_check>.2:
                if not self._sources_valid(self.bundle):return False
                self.last_source_check=time.monotonic()
            if not np.isfinite([body[k] for k in ['height','waist','head_yaw','head_pitch']]).all():return False
            if self.source_identity is not None and any((Path(p).stat().st_ino,Path(p).stat().st_size,Path(p).stat().st_mtime_ns)!=self.source_identity[k] for k,p in self.sources.items()):return False
            if s['chassis_active'] or not body['fresh'] or (starting and s['motion_reserved']):return False
            if abs(body['height']-ref['height'])>.03:return False
            if any(abs(body[k]-ref[k])>.002 for k in ['waist','head_yaw','head_pitch']):return False
            for side in ['left','right']:
                arm=s['arms'][side];base=initial['arms'][side];q=np.asarray(arm['joints'],float)
                if not math.isfinite(arm['gripper']) or not arm['fresh'] or q.shape!=(6,) or not np.isfinite(q).all() or abs(arm['gripper']-base['gripper'])>.05:return False
                if (starting or side=='left') and np.max(abs(q-np.asarray(base['joints'])))>np.radians(.25):return False
            if not -.15<=s['arms']['right']['gripper']<=.03:return False
            if self.token is not None and not self.ctrl.motion_valid(self.token):return False
            if self.reference is not None:
                plan,started=self.reference;q=self._point(plan,max(0,time.monotonic()-started));actual=np.asarray(s['arms']['right']['joints'])
                if np.max(abs(q-actual))>np.radians(1):return False
                model=self.ctrl.models['right'];a=model.wrist_transform(actual,body['height']);b=model.wrist_transform(q,body['height'])
                tcp=np.asarray(self.bundle['tcp_matrix']);pa=(a@tcp)[:3,3];pb=(b@tcp)[:3,3]
                if np.linalg.norm(pa-pb)>.001:return False
                angle=math.acos(float(np.clip((np.trace(a[:3,:3].T@b[:3,:3])-1)/2,-1,1)))
                if angle>np.radians(.3):return False
            return True
        except (KeyError,ValueError,TypeError,AttributeError,OSError):return False
    @staticmethod
    def _point(plan,elapsed):
        ts=np.asarray(plan['time_from_start_s']);qs=np.asarray(plan['joint_waypoints_rad']);i=min(max(0,np.searchsorted(ts,elapsed,side='right')-1),len(ts)-2);u=np.clip((elapsed-ts[i])/(ts[i+1]-ts[i]),0,1);s=u**3*(10+u*(-15+6*u));return qs[i]+s*(qs[i+1]-qs[i])
    def action(self,data):
        with self.lock:
            if data=={'op':'cancel'}:
                self.cancel.set();return self.snapshot()
            if not isinstance(data,dict) or set(data)!={'op','run_id'} or data['op']!='start':raise ValueError('仅允许本次固定空载路线的开始或取消')
            if self.thread is not None and self.thread.is_alive():raise ValueError('空载测试已运行')
            b=self._load(data['run_id']);self.bundle=b;self.reference=None;s=self.ctrl.snapshot()
            if not self._guard(s,0,starting=True):raise ValueError('现场反馈、起姿或保持状态与冻结计划不一致')
            start=np.asarray(b['stages'][0]['plan']['joint_waypoints_rad'][0]);actual=np.asarray(s['arms']['right']['joints'])
            if np.max(abs(start-actual))>np.radians(.25):raise ValueError('轨迹首点不等于真实起姿')
            self.directory.mkdir(parents=True,exist_ok=True)
            try:
                with (self.directory/('consumed-'+b['run_id'])).open('x') as h:h.write(str(time.time()));h.flush();__import__('os').fsync(h.fileno())
            except FileExistsError as e:raise ValueError('本次空载计划已使用，禁止重放') from e
            self.token=self.ctrl.reserve_motion(s['command_epoch']);self.cancel.clear();self.done.clear();self._save(status='running',phase='starting',run_id=b['run_id'],reason='',recovery_required=False)
            self.thread=threading.Thread(target=self._run,name='empty-cabinet-rehearsal',daemon=True)
            try:self.thread.start()
            except Exception:
                self.ctrl.release_motion(self.token);self.token=None;raise
            return self.snapshot()
    def _arrived(self,target=None):
        after=time.time();deadline=time.monotonic()+self.arrival_timeout_s
        previous=None;first=None;count=0;seen={side:after for side in ['left','right','body']}
        while time.monotonic()<deadline:
            s=self.ctrl.snapshot();arm=s['arms']['right'];body=s['body'];left=s['arms']['left']
            rows={'right':arm,'left':left,'body':body}
            try:
                if not all(r['fresh'] and math.isfinite(r['timestamp']) and r['timestamp']<=time.time()+.1 for r in rows.values()):return False
                if any(rows[k]['timestamp']<=seen[k] for k in seen):
                    time.sleep(self.arrival_interval_s);continue
                q=np.asarray(arm['joints'],float);other=np.asarray(left['joints'],float);pose=np.asarray([body[k] for k in ['height','waist','head_yaw','head_pitch']],float)
                if q.shape!=(6,) or other.shape!=(6,) or not np.isfinite(np.r_[q,other,pose]).all():return False
                if target is not None and np.max(abs(q-np.asarray(target)))>np.radians(.25):return False
                measured=np.r_[q,other,pose]
                if previous is not None and np.max(abs(measured-previous))>np.radians(.05):return False
                seen={k:rows[k]['timestamp'] for k in seen};count+=1;first=first or min(seen.values());previous=measured
                if count>=self.arrival_samples and min(seen.values())-first>=self.arrival_interval_s*(self.arrival_samples-1):return True
            except (KeyError,ValueError,TypeError):return False
            time.sleep(self.arrival_interval_s)
        return False
    def _run(self):
        stopped=True
        try:
            if not self._arrived(self.bundle['initial']['arms']['right']['joints']):raise ValueError('启动前没有收到连续的新反馈证明停稳')
            for stage in self.bundle['stages']:
                if self.cancel.is_set() or not self._sources_valid(self.bundle) or not self._guard(self.ctrl.snapshot(),0):raise ValueError('取消或现场反馈已变化')
                plan=stage['plan'];s=self.ctrl.snapshot()
                if np.max(abs(np.asarray(s['arms']['right']['joints'])-plan['joint_waypoints_rad'][0]))>np.radians(.25):raise ValueError('分段首点与实测不符')
                self._save(phase=stage['name']);self.reference=(plan,time.monotonic());stopped=False
                self.ctrl.start_trajectory('right',plan,self.token,preflight=self._guard);deadline=time.monotonic()+plan['time_from_start_s'][-1]+8
                while True:
                    if self.cancel.wait(self.poll_s):raise ValueError('现场请求取消')
                    if time.monotonic()>deadline:raise ValueError('轨迹执行超时')
                    receipt=self.ctrl.poll_trajectory(self.token,preflight=self._guard)
                    if receipt['status']=='reference_complete':break
                self.reference=None
                if not self._arrived(plan['joint_waypoints_rad'][-1]) or not self._guard(self.ctrl.snapshot(),0):raise ValueError('参考完成但实测到位或停稳检查未通过')
                stopped=True
            self._save(status='completed',phase='outside_hold',reason='空载浅入并原路撤回，未放置电池')
        except Exception as e:
            self.reference=None
            try:self.ctrl.submit({'type':'arm.stop','arm':'right'},self.token)
            except Exception as stop_error:self._save(stop_error=str(stop_error)[:200])
            stopped=self._arrived()
            self._save(status='stopped' if stopped else 'recovery_required',reason=str(e)[:250],recovery_required=not stopped)
        finally:
            self.reference=None
            if stopped and self.token is not None:
                try:self.ctrl.release_motion(self.token);self.token=None
                except Exception as e:self._save(status='recovery_required',reason=str(e)[:200],recovery_required=True)
            self.done.set()
    def wait(self,timeout):return self.done.wait(timeout)
    def close(self):
        self.cancel.set()
        if self.thread is not None:self.thread.join(max(4,2*self.arrival_timeout_s+2))
