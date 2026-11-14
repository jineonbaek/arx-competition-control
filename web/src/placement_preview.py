"""Reuse the existing RGB-D owner; analyze a single red ring without motion."""
import base64
import hashlib
import json
from pathlib import Path
import threading
import cv2
import numpy as np
from agent.placement_geometry import analyze
from agent.storage import atomic_json

class PlacementPreview:
    def __init__(self,controller,depth,*,captures=None):
        if captures is None:
            from grasp_preview import GraspPreview
            captures={c:GraspPreview(controller,depth,camera=c) for c in ('d435','d405')}
        self.captures=captures;self.lock=threading.Lock();self.latest={};self.sources={}

    def capture(self,camera,store=None,*,allow_occupied=False,settings=None):
        if camera not in ('d435','d405'):raise ValueError('仅支持D435和D405配对深度')
        if not self.lock.acquire(False):raise ValueError('放置视觉正在采集中')
        try:
            self.latest.pop(camera,None)
            capture=self.captures[camera];view=capture.handle_action({'op':'capture'})
            blob=(capture.cache/'frame.npz').read_bytes()
            import io
            with np.load(io.BytesIO(blob),allow_pickle=False) as f:
                frame={k:f[k].copy() for k in f.files}
            if str(frame['id'])!=view['id']:raise ValueError('放置冻结帧已被更新')
            r=analyze(frame,settings,allow_occupied=allow_occupied);r['scene_sha256']=hashlib.sha256(blob).hexdigest()
            if store is not None:
                from agent.grasp_scene import save_frame
                obs=save_frame(store,blob,source='red_ring_placement',expected_id=view['id'],
                    calibration_sha256=str(frame['calibration_sha256']),camera=camera)
                r['observation_id']=obs['observation_id'];atomic_json(store.path/(camera+'-placement.json'),r)
            self.sources[r['snapshot_id']]=frame
            if len(self.sources)>4:self.sources.pop(next(iter(self.sources)))
            image=frame['rgb'][:,:,::-1].copy();contour=np.array(r['ring_contour_px'],np.int32)
            cv2.polylines(image,[contour],True,(0,230,250),2)
            world=frame['xyz']@frame['base_from_color'][:3,:3].T+frame['base_from_color'][:3,3]
            valid=frame['xyz'][:,:,2]>.07
            def project(point):
                distance=np.linalg.norm(world-np.asarray(point),axis=2);distance[~valid]=np.inf
                y,x=np.unravel_index(np.argmin(distance),distance.shape);return int(x),int(y)
            poly=np.array([project(p) for p in r['footprint_base_m']],np.int32)
            cv2.polylines(image,[poly],True,(0,240,0),2);point=project(r['target_base_m'])
            cv2.drawMarker(image,point,(0,240,0),cv2.MARKER_CROSS,20,2)
            cv2.putText(image,'LEFT / PLACE',point,cv2.FONT_HERSHEY_SIMPLEX,.5,(0,240,0),1)
            ok,jpeg=cv2.imencode('.jpg',image)
            if not ok:raise ValueError('标注图生成失败')
            self.latest[camera]={**r,'image':'data:image/jpeg;base64,'+base64.b64encode(jpeg).decode()}
            return dict(self.latest[camera])
        finally:self.lock.release()

    def action(self,data):
        if not isinstance(data,dict) or set(data)!={'camera'}:raise ValueError('只接受相机名称，不能指定坐标、执行权限或容差')
        return self.capture(data['camera'])

    def check_payload_support(self,result,base_from_battery,bounds):
        """Fresh pre-release occupancy may contain only the known held CAD box."""
        from agent.placement_geometry import rigid
        frame=self.sources[result['snapshot_id']];xyz=frame['xyz'];camera=rigid(frame['base_from_color']);held=rigid(base_from_battery)
        valid=(xyz[:,:,2]>.07)&(xyz[:,:,2]<3.);pts=xyz[valid]@camera[:3,:3].T+camera[:3,3]
        delta=pts-result['target_base_m'];extent=result['verified_half_extent_m'];height=delta@np.asarray(result['normal_base'])
        relevant=(abs(delta@np.asarray(result['forward_axis_base']))<extent[0])&(abs(delta@np.asarray(result['left_axis_base']))<extent[1])&(height>.01)
        local=(pts-held[:3,3])@held[:3,:3];bounds=np.asarray(bounds)
        own=np.all((local>=bounds[0]-.005)&(local<=bounds[1]+.005),axis=1)
        if np.count_nonzero(relevant&~own)>10:raise ValueError('松爪前发现持物模型外的新障碍')

    def verify_battery(self,store,result,expected,settings,*,recognize=True):
        """Same battery recognition stack as grasp; new post-release frames only."""
        from agent.grasp_pipeline import segment,gpu_slot
        from agent.grasp_scene import frame_path
        from PIL import Image
        ident=result['observation_id'];output=store.path/'placement-verification'/ident;output.mkdir(parents=True)
        if recognize:
            with gpu_slot():report=segment(store,ident,output,lambda:store.cancelled)
            if report.get('status')!='perception_accepted':raise ValueError('放置后未确认识别到电池')
        with np.load(frame_path(store,ident),allow_pickle=False) as f:
            xyz=f['xyz'];t=f['base_from_color'];valid=(xyz[:,:,2]>.07)&(xyz[:,:,2]<3.)
            if recognize:valid&=np.asarray(Image.open(output/'mask.png').convert('L'))>0
            pts=xyz[valid]@t[:3,:3].T+t[:3,3]
        if not recognize:
            delta=pts-expected['target_base_m'];size=np.array(settings['battery_size_m']);tol=expected['xy_tolerance_mm']/1000
            mask=(abs(delta@np.asarray(expected['forward_axis_base']))<size[0]/2+tol)&(abs(delta@np.asarray(expected['left_axis_base']))<size[1]/2+tol)
            height=delta@np.asarray(expected['normal_base']);pts=pts[mask&(height>.02)&(height<size[2]+.03)]
        h=(pts-expected['target_base_m'])@np.asarray(expected['normal_base'])
        pts=pts[h>.02]
        if len(pts)<100:raise ValueError('电池识别区域有效深度不足')
        d=pts-expected['target_base_m'];u=d@np.array(expected['forward_axis_base']);v=d@np.array(expected['left_axis_base']);z=d@np.array(expected['normal_base'])
        size=np.array(settings['battery_size_m']);tol=expected['xy_tolerance_mm']/1000
        inside=(abs(u)<size[0]/2+tol)&(abs(v)<size[1]/2+tol)&(z>-.003)&(z<size[2]+.03)
        if inside.mean()<.8 or abs(float(np.percentile(z[inside],95))-size[2])>.04:
            raise ValueError('识别电池未落在圈内左侧或高度不符')
        return dict(center_base_m=np.median(pts[inside],axis=0).tolist(),battery_recognized=True)
