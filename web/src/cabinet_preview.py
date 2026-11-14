"""Cabinet recognition and path preview only. No motion reservation/submission API."""
import base64
import json
import threading
from pathlib import Path
import time
import cv2
import numpy as np
from agent.cabinet_preview_geometry import detect,localize,opening,horizontal_plan
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
        self.controller=controller;self.root=Path(root);self.lock=threading.Lock();self.latest={}
        self.profiles=json.loads((self.root/'config/cabinet-profiles.json').read_text())
        self.battery=json.loads((self.root/'config/battery-envelope.json').read_text())
        self.grip=json.loads((self.root/'web/public/cad/battery-grip.json').read_text())
        if captures is None:
            from grasp_preview import GraspPreview
            captures={camera:GraspPreview(controller,depth,cache=self.root/'web/hardware/runtime'/('cabinet-'+camera),camera=camera) for camera in ('d435','d405')}
        self.captures=captures

    def action(self,data):
        if not isinstance(data,dict) or set(data)!={'camera','model'} or data['camera'] not in self.captures or data['model'] not in ('demo','competition'):
            raise ValueError('仅接受相机与电池仓模型选择，不接受坐标或执行参数')
        state=self.controller.snapshot()
        if state.get('motion_reserved') is not False or state.get('chassis_active') is not False:raise ValueError('请先结束运动任务再拍摄规划预览')
        if not self.lock.acquire(False):raise ValueError('仓口识别/路径规划正在进行')
        camera=data['camera'];self.latest.pop(camera,None)
        try:
            capture=self.captures[camera];view=capture.handle_action({'op':'capture'})
            with np.load(capture.cache/'frame.npz',allow_pickle=False) as source:frame={k:source[k].copy() for k in source.files}
            if str(frame['id'])!=view['id']:raise ValueError('配对深度帧已被更新')
            return self.analyze_frame(camera,data['model'],frame)
        finally:self.lock.release()

    def analyze_frame(self,camera,mode,frame,*,now=None):
        if camera not in ('d435','d405') or str(frame['camera_id'])!=camera:raise ValueError('相机身份与配对帧不一致')
        detected=detect(frame['rgb']);image=frame['rgb'][:,:,::-1].copy()
        result={'camera':camera,'model':mode,'snapshot_id':str(frame['id']),'image_time':float(frame['image_time']),
            'status':detected['status'],'executable':False,'reason':detected.get('reason','')}
        if detected['status']!='complete':
            for x,y,right,bottom in detected.get('partial_boxes_px',[]):cv2.rectangle(image,(x,y),(right-1,bottom-1),(0,190,255),2)
        else:
            result['detected_quad_px']=detected['inner_quad_px']
            cv2.polylines(image,[np.rint(detected['inner_quad_px']).astype(np.int32)],True,(255,230,0),2)
            cv2.polylines(image,[np.array(detected['outer_contour_px'],np.int32)],True,(0,200,255),1)
            slot=opening(self.profiles,mode)
            try:
                window=localize(frame,slot,now=now)
                result.update(status='localized',window=window,target_slot=slot['slot_id'])
                cv2.polylines(image,[np.rint(window['inner_quad_px']).astype(np.int32)],True,(255,230,0),2)
                cv2.polylines(image,[np.array(window['outer_contour_px'],np.int32)],True,(0,200,255),1)
                plan=horizontal_plan(window,slot,self.battery,tip_point_object_mm=self.grip['points_object_mm']['tip'])
                result['plan']=plan
                result['alignment']=self._alignment(frame,plan)
                result['arm_path']=self._arm_preview(frame,plan)
                result['observed_corridor_points']=self._corridor(frame,window,plan)
                if window['rim_plane_p90_mm']>=plan['nominal_cad_clearance_mm']:
                    plan['blockers'].append('红框深度平面P90残差大于CAD最小净距，当前不能作为插入精度证据')
                self._draw(image,frame,window,plan)
                if result['observed_corridor_points']>15:plan['blockers'].append('实测点云在入仓包络中有潜在碰撞点，需复核')
            except (ValueError,KeyError) as e:result.update(status='planning_rejected' if 'window' in result else 'depth_rejected',reason=str(e))
        ok,jpeg=cv2.imencode('.jpg',image)
        if not ok:raise ValueError('仓口预览图生成失败')
        result['image']='data:image/jpeg;base64,'+base64.b64encode(jpeg).decode()
        self.latest[camera]=result
        directory=self.root/'web/hardware/runtime/cabinet-preview';directory.mkdir(parents=True,exist_ok=True)
        atomic_json(directory/(camera+'-result.json'),result)
        return result

    def _alignment(self,frame,plan):
        try:
            model=self.controller.models['right'];tcp=np.array(json.loads((self.root/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
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
            model=SeededModel(self.controller.models['right']);current=float(frame['lift_rad'])
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
        k=json.loads(str(frame['intrinsics']));points=[]
        # Captures contain factory undistorted rays; display projection is only an overlay.
        for x,y,z in p:
            points.append([int(round(k['fx']*x/z+k['cx'])),int(round(k['fy']*y/z+k['cy']))] if z>.07 else None)
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
