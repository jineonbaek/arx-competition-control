"""Cabinet recognition and path preview only. No motion reservation/submission API."""
import base64
import hashlib
import json
import threading
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import cv2
import numpy as np
from agent.cabinet_preview_geometry import detect,localize,opening,horizontal_plan
from agent.cabinet_rgb_geometry import localize_rgb,camera_matrix
from cabinet_capture import CabinetColorCapture,CALIBRATIONS
from agent.storage import atomic_json

ROOT=Path(__file__).resolve().parents[2]

class SeededModel:
    """Alternative solver seeds only; every result is swept from the real start."""
    def __init__(self,model):self.model=model
    def __getattr__(self,name):return getattr(self.model,name)
    def inverse_wrist(self,q,lift,xyz,rpy,limits):
        errors=[];bounds=np.radians(np.asarray(limits,float))
        seeds=[q,[0,.5,1.,0,.5,-1.57],[0,.8,1.5,0,.5,-1.57],[0,1.,2.,0,1.,-1.57]]
        for seed in seeds:
            seed=np.clip(seed,bounds[:,0]+1e-6,bounds[:,1]-1e-6)
            try:
                result=self.model.inverse_wrist(seed,lift,xyz,rpy,limits)
                if self.model.check(q,result,lift)['clear']:return result
                errors.append('实际起姿到候选解的CAD扫掠未通过')
            except ValueError as e:errors.append(str(e))
        raise ValueError(errors[-1] if errors else '没有逆解候选')

class CabinetPreview:
    def __init__(self,controller,depth,*,captures=None,root=ROOT):
        self.controller=controller;self.root=Path(root);self.lock=threading.Lock();self.cache_lock=threading.Lock();self.latest={};self.bindings={}
        self.profiles=json.loads((self.root/'config/cabinet-profiles.json').read_text())
        self.battery=json.loads((self.root/'config/battery-envelope.json').read_text())
        self.grip=json.loads((self.root/'web/public/cad/battery-grip.json').read_text())
        if captures is None:
            captures={camera:CabinetColorCapture(controller,camera=camera,root=self.root) for camera in ('d435','d405')}
        self.captures=captures

    def action(self,data,*,include_arm_preview=True,token=None):
        if not isinstance(data,dict) or set(data)!={'camera','model'} or data['camera'] not in self.captures or data['model'] not in ('demo','competition'):
            raise ValueError('仅接受相机与电池仓模型选择，不接受坐标或执行参数')
        state=self.controller.snapshot()
        owned=token is not None and self.controller.motion_valid(token)
        if (state.get('motion_reserved') is not False and not owned) or state.get('chassis_active') is not False:raise ValueError('请先结束运动任务再拍摄规划预览')
        if not self.lock.acquire(False):raise ValueError('仓口识别/路径规划正在进行')
        camera=data['camera']
        with self.cache_lock:self.latest.pop(camera,None)
        try:
            detection=None
            if owned and not include_arm_preview:
                detection,frame=self.captures[camera].capture_consensus(token=token)
            else:frame=self.captures[camera].capture(token=token) if owned else self.captures[camera].capture()
            result=self.analyze_frame(camera,data['model'],frame,include_arm_preview=include_arm_preview,detection=detection)
            # A task owns the motion token while planning; browser previews keep their idle binding.
            return result if owned else self.snapshot()[camera]
        finally:self.lock.release()

    def analyze_frame(self,camera,mode,frame,*,now=None,include_arm_preview=True,detection=None):
        if camera not in ('d435','d405') or str(frame['camera_id'])!=camera:raise ValueError('相机身份与配对帧不一致')
        hashes=self._source_hashes(camera)
        self.profiles=json.loads((self.root/'config/cabinet-profiles.json').read_text())
        self.battery=json.loads((self.root/'config/battery-envelope.json').read_text())
        self.grip=json.loads((self.root/'web/public/cad/battery-grip.json').read_text())
        detected=detect(frame['rgb']) if detection is None else dict(detection);image=frame['rgb'][:,:,::-1].copy()
        result={'camera':camera,'role':'primary' if camera=='d435' else 'auxiliary','model':mode,
            'snapshot_id':str(frame['id']),'image_time':float(detected.get('consensus_time',frame['image_time'])),
            'latest_exposure_time':float(frame['image_time']),
            'status':detected['status'],'executable':False,'reason':detected.get('reason',''),
            'environment_evidence':{'status':'unavailable','reason':'点云未配置，本链路不使用深度碰撞结论'},
            'readiness':{'execution_ready':False,'blockers':['尚未验收负载路径、入仓精度和松爪条件']}}
        if detected['status']!='complete':
            for x,y,right,bottom in detected.get('partial_boxes_px',[]):cv2.rectangle(image,(x,y),(right-1,bottom-1),(0,190,255),2)
        else:
            result['detected_quad_px']=detected['inner_quad_px']
            cv2.polylines(image,[np.rint(detected['inner_quad_px']).astype(np.int32)],True,(255,230,0),2)
            cv2.polylines(image,[np.array(detected['outer_contour_px'],np.int32)],True,(0,200,255),1)
            slot=opening(self.profiles,mode)
            try:
                window=localize_rgb(frame,slot,now=now,detection=detected)
                result.update(status='localized',window=window,target_slot=slot['slot_id'])
                try:
                    from agent.cabinet_loaded_sweep import infer_table
                    mesh=(self.root/'vision/models/cabinet'/(mode+'.stl')).read_bytes()
                    result['table_estimate']=infer_table(frame,window,self.profiles[mode],slot,mesh)
                except (ValueError,KeyError,OSError) as error:
                    result['table_estimate']={'status':'unavailable','reason':str(error)}
                result['readiness']['blockers'].append(window['scale_assumption'])
                if window['pose_ambiguous']:result['readiness']['blockers'].append('平面PnP存在近似等价姿态，预览选最低像素误差候选')
                cv2.polylines(image,[np.rint(window['inner_quad_px']).astype(np.int32)],True,(255,230,0),2)
                cv2.polylines(image,[np.array(window['outer_contour_px'],np.int32)],True,(0,200,255),1)
                plan=horizontal_plan(window,slot,self.battery,tip_point_object_mm=self.grip['points_object_mm']['tip'])
                result['plan']=plan
                result['alignment']=self._alignment(frame,plan)
                result['arm_path']=(self._arm_preview(frame,plan) if include_arm_preview else
                    dict(status='deferred',executable=False,reason='实机任务独立计算当前升降的轨迹；未重复运行预览逆解'))
                plan['blockers'].extend(result['readiness']['blockers'])
                self._draw(image,frame,window,plan)
            except (ValueError,KeyError) as e:result.update(status='planning_rejected' if 'window' in result else 'rgb_rejected',reason=str(e))
        status=str(frame.get('calibration_status','未声明验证状态'))
        result['calibration']={'camera_sha256':str(frame.get('calibration_sha256','')),
            'camera_status':status,'physical_base_accuracy_verified':False,
            'tcp_sha256':hashes.get('config/right-grip-tcp.json'),
            'note':'沿用原抓取TCP；相机到机器人基座的精度需独立验证'}
        result['readiness']['blockers'].append('相机到基座转换未完成实测验收：'+status)
        if result.get('plan'):result['plan']['blockers'].append(result['readiness']['blockers'][-1])
        result['source_hashes']=hashes;result['expires_at']=float(frame['image_time'])+30.
        result['validity']={'valid':True,'reason':'冻结帧预览，需持续核对状态'}
        bound={key:np.asarray(frame[key]).copy() if key in frame else None for key in ('joints','lift_rad','right_gripper','left_joints','command_epoch','body_configuration')}
        captured_calibration_matches=hashes[CALIBRATIONS[camera]] in (None,str(frame.get('calibration_sha256','')))
        if self._source_hashes(camera)!=hashes or not captured_calibration_matches:
            result={k:v for k,v in result.items() if k not in ('plan','window','alignment','arm_path')}
            result.update(status='stale',reason='采集后模型或标定发生变化，请重新识别',validity={'valid':False})
        ok,jpeg=cv2.imencode('.jpg',image)
        if not ok:raise ValueError('仓口预览图生成失败')
        result['image']='data:image/jpeg;base64,'+base64.b64encode(jpeg).decode()
        with self.cache_lock:self.bindings[camera]=bound;self.latest[camera]=result
        directory=self.root/'web/hardware/runtime/cabinet-preview';directory.mkdir(parents=True,exist_ok=True)
        atomic_json(directory/(camera+'-result.json'),result)
        return result

    def _source_hashes(self,camera):
        paths=[CALIBRATIONS[camera],'config/right-grip-tcp.json','config/cabinet-profiles.json',
            'config/battery-envelope.json','web/public/cad/battery-grip.json',
            'agent/cabinet_consensus.py','agent/cabinet_red_edges.py','agent/cabinet_loaded_sweep.py','vision/models/cabinet/demo.stl','agent/cabinet_rgb_geometry.py','agent/cabinet_preview_geometry.py','agent/grasp_path.py',
            'web/src/cabinet_preview.py','web/src/cabinet_capture.py','web/src/handeye_column.py','web/config/control.json']
        package=Path('web/public/models/lift2/diy-20261105');urdf=package/'urdf/lift2_with_sensors.urdf';paths.append(str(urdf))
        if (self.root/urdf).is_file():
            xml=ET.parse(self.root/urdf)
            for node in xml.findall('./link/visual/geometry/mesh'):
                name=node.get('filename','')
                if name.startswith('package://lift2/'):paths.append(str(package/name.removeprefix('package://lift2/')))
        model=getattr(self.controller,'models',{}).get('right')
        for name in getattr(model,'source_hashes',{}):paths.append(name)
        return {p:hashlib.sha256((self.root/p).read_bytes()).hexdigest() if (self.root/p).is_file() else None for p in paths}

    def _checked_model(self):
        model=self.controller.models['right'];sources=getattr(model,'source_hashes',{})
        if not sources:raise ValueError('机械臂模型没有加载来源身份，需重新加载模型后规划')
        if any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h for p,h in sources.items()):
            raise ValueError('机械臂模型来源已变化，旧模型不能继续计算坐标或路径')
        return model

    def _invalid_reason(self,result,bound,state,now):
        try:
            if result.get('validity',{}).get('valid') is not True:return result.get('reason','预览已失效')
            if not np.isfinite(now) or not -.1<=now-result['image_time']<=30:return '冻结预览已过期，请重新识别'
            if state.get('motion_reserved') is not False or state.get('chassis_active') is not False:return '机器人已开始运动，旧路径已失效'
            arm=state['arms']['right'];body=state['body']
            stamps=np.array([arm['timestamp'],body['timestamp']],float)
            if not arm.get('fresh') or not body.get('fresh') or not np.isfinite(stamps).all() or np.max(abs(now-stamps))>.5:return '机器人反馈过期，旧路径已失效'
            if bound['command_epoch'] is None or state.get('command_epoch')!=int(bound['command_epoch']):return '控制指令已变化，请重新识别'
            q=np.asarray(arm['joints'],float);old=np.asarray(bound['joints'],float)
            if q.shape!=(6,) or old.shape!=(6,) or not np.isfinite(q).all() or not np.isfinite(old).all():return '关节绑定无效，请重新识别'
            scalars=np.array([body['height'],bound['lift_rad'],arm['gripper'],bound['right_gripper']],float)
            if not np.isfinite(scalars).all():return '升降或夹持绑定无效，请重新识别'
            if np.max(abs(q-old))>np.radians(.25) or abs(scalars[0]-scalars[1])>.05 or abs(scalars[2]-scalars[3])>.05:return '关节、升降或夹持位置已变化，请重新识别'
            if bound['left_joints'] is not None:
                left=state['arms']['left'];lq=np.asarray(left['joints'],float);previous=np.asarray(bound['left_joints'],float)
                if (not left.get('fresh') or abs(now-left['timestamp'])>.5 or lq.shape!=(6,) or previous.shape!=(6,)
                        or not np.isfinite(lq).all() or not np.isfinite(previous).all() or np.max(abs(lq-previous))>np.radians(.25)):
                    return '左臂状态已变化，请重新识别'
            if bound['body_configuration'] is not None:
                for key,value in json.loads(str(bound['body_configuration'])).items():
                    if value is None:
                        if body.get(key) is not None:return '机体或相机姿态已变化，请重新识别'
                    elif (body.get(key) is None or not np.isfinite([value,body[key]]).all() or abs(body[key]-value)>.005):
                        return '机体或相机姿态已变化，请重新识别'
            if self._source_hashes(result['camera'])!=result['source_hashes']:return '模型、TCP、标定或规划代码已变化，请重新识别'
            return ''
        except (KeyError,TypeError,ValueError,OSError,OverflowError):return '缺少完整状态绑定，请重新识别'

    def snapshot(self,*,now=None):
        now=time.time() if now is None else now;state=self.controller.snapshot();out={}
        with self.cache_lock:
            for camera,result in self.latest.items():
                reason=self._invalid_reason(result,self.bindings.get(camera,{}),state,now)
                if reason:
                    result={k:v for k,v in result.items() if k not in ('plan','window','alignment','arm_path')}
                    result.update(status='stale',reason=reason,validity={'valid':False,'reason':reason},executable=False)
                    self.latest[camera]=result
                out[camera]=result
        return out

    def _alignment(self,frame,plan):
        try:
            model=self._checked_model();tcp=np.array(json.loads((self.root/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
            current=model.wrist_transform(frame['joints'],float(frame['lift_rad']))@tcp
            return dict(current_tcp_base_mm=(current[:3,3]*1000).tolist(),pre_tcp_base_mm=plan['tcp_pre_base_mm'],
                outside_alignment_required=True,alignment_changes_z=True,
                delta_z_mm=plan['fixed_z_mm']-float(current[2,3]*1000),note='仓外对齐独立于固定Z入仓段，不发送指令')
        except (ValueError,KeyError,OSError):return dict(status='unavailable',outside_alignment_required=True)

    def _arm_preview(self,frame,plan):
        from agent.grasp_path import plan_insert
        attempts=[]
        try:
            tcp=np.array(json.loads((self.root/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
            model=SeededModel(self._checked_model());current=float(frame['lift_rad'])
            # Conditional outside-alignment alternatives, never lift commands.
            candidates=[current]+sorted([v for v in (10.,8.,5.52) if abs(v-current)>.05],key=lambda v:abs(v-current))
            for lift in candidates:
                try:
                    result=plan_insert(model,frame['joints'],lift,plan['tcp_pre_base_mm'],plan['tcp_end_base_mm'],
                        plan['R_base_from_tool'],tcp,self.controller.config['arms']['right']['joint_limits_deg'])
                    result.update(executable=False,planning_lift_rad=lift,current_lift_rad=current,
                        outside_lift_alignment_required=abs(lift-current)>.05,both_arms_lift_sweep_checked=False,attempts=attempts)
                    if abs(lift-current)>.05:result['blockers'].append('候选升降高度仅供仓外对齐预览，双臂升降沿途与环境检查尚未完成')
                    return result
                except ValueError as e:attempts.append({'lift_rad':lift,'reason':str(e)})
            return dict(status='rejected',reason=attempts[-1]['reason'],attempts=attempts,executable=False)
        except (ValueError,KeyError,OSError) as e:return dict(status='rejected',reason=str(e),executable=False)

    @staticmethod
    def _project(frame,points_mm):
        t=frame['base_from_color'];p=(np.asarray(points_mm)/1000-t[:3,3])@t[:3,:3]
        k=json.loads(str(frame['intrinsics']));matrix,dist=camera_matrix(k);points=[]
        for x,y,z in p:
            uv=cv2.projectPoints(np.array([[x,y,z]],float),np.zeros(3),np.zeros(3),matrix,dist)[0].reshape(2) if z>.07 else None
            points.append(np.rint(uv).astype(int).tolist() if uv is not None else None)
        return points

    def _draw(self,image,frame,window,plan):
        for key,color in [('tcp_points_base_mm',(255,120,20)),('leading_points_base_mm',(0,220,0))]:
            points=self._project(frame,plan[key]);valid=[p for p in points if p is not None]
            if len(valid)>1:cv2.polylines(image,[np.array(valid,np.int32)],False,color,2);cv2.arrowedLine(image,tuple(valid[-2]),tuple(valid[-1]),color,2,tipLength=.25)
        center=np.array(window['center_base_m']);left=np.array(window['left_axis_base']);point=center+left*(plan['effective_opening_size_mm'][0]/4000)
        pixel=self._project(frame,[point*1000])[0]
        if pixel:
            cv2.drawMarker(image,tuple(pixel),(0,240,0),cv2.MARKER_CROSS,14,2)
            cv2.putText(image,'LEFT / FIXED Z',(pixel[0]-40,pixel[1]-12),cv2.FONT_HERSHEY_SIMPLEX,.42,(0,180,0),1)

    @staticmethod
    def _corridor(frame,window,plan):
        xyz=frame['xyz'];valid=(xyz[:,:,2]>.07)&(xyz[:,:,2]<3.);t=frame['base_from_color'];pts=xyz[valid]@t[:3,:3].T+t[:3,3]
        center=np.asarray(window['center_base_m']);I=-np.asarray(window['outward_normal_base']);L=np.asarray(window['left_axis_base']);V=np.asarray(window['up_axis_base']);delta=pts-center
        depth=delta@I*1000;uv=np.column_stack((delta@L,delta@V))*1000
        uv=uv[(depth>10)&(depth<plan['leading_insertion_depth_mm']-5)]
        if not len(uv):return 0
        hull=cv2.convexHull(np.asarray(plan['aperture_envelope_samples_mm'],np.float32))[:,0,:].astype(float)
        edge=np.roll(hull,-1,axis=0)-hull;rel=uv[None,:,:]-hull[:,None,:]
        cross=edge[:,0,None]*rel[:,:,1]-edge[:,1,None]*rel[:,:,0]
        winding=np.sum(hull[:,0]*np.roll(hull[:,1],-1)-hull[:,1]*np.roll(hull[:,0],-1))
        return int(np.count_nonzero(np.all(cross>=-1e-6,axis=0) if winding>0 else np.all(cross<=1e-6,axis=0)))

    def verify_placement(self,store,observation,token,check,sleep):
        from cabinet_verification import verify_placement
        return verify_placement(self,store,observation,token,check,sleep)
