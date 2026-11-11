"""One business tool: freeze a camera, recognize the target and localize its surface."""
from pathlib import Path
import numpy as np
from PIL import Image
from .contracts import AgentError
from .grasp_scene import capture, frame_path
from .grasp_geometry import validate_frame
from .grasp_pipeline import segment, gpu_slot
from .storage import atomic_json


def surface_location(frame, mask):
    transform=validate_frame(frame)
    xyz=np.asarray(frame['xyz'])
    mask=np.asarray(mask)
    if mask.shape!=xyz.shape[:2] or not np.isfinite(mask).all():
        raise ValueError('目标掩膜与冻结深度不匹配')
    foreground=mask>0
    valid=foreground & (xyz[...,2]>.07) & (xyz[...,2]<3.)
    points=xyz[valid]
    if len(points)<100 or len(points)<.5*max(1,int(foreground.sum())):
        raise ValueError('电池区域有效深度不足')
    order=np.argsort(points[:,2]);points=points[order]
    # Separate clearly disconnected depth layers; never average a foreground and its hole background.
    groups=np.split(points,np.flatnonzero(np.diff(points[:,2])>.08)+1)
    surface=max(groups,key=len)
    if len(surface)<.6*len(points):raise ValueError('电池区域存在多个有歧义的深度层')
    base=surface@transform[:3,:3].T+transform[:3,3]
    spread=np.percentile(base,95,axis=0)-np.percentile(base,5,axis=0)
    if max(spread)>.30:raise ValueError('深度范围超过电池尺寸，可能包含背景')
    return dict(frame='base_link',unit='mm',kind='visible_surface_not_grip',
        surface_base_mm=(np.median(base,axis=0)*1000).tolist(),
        surface_spread_p90_mm=(spread*1000).tolist(),support_pixels=len(surface),
        valid_pixels=len(points),mask_pixels=int(foreground.sum()),
        precision_validated=False,
        note='可见表面粗位置，不是提手夹持点；升降后必须由 D405 重测。')


def observe(registry,camera,call_id):
    registry.progress('observe_'+camera)
    registry.store.state.pop('grasp_preview',None)
    latest=registry.store.state.setdefault('latest_battery_observations',{})
    latest.pop(camera,None)
    if camera=='d435':
        registry.store.state.pop('coarse_alignment',None)
        latest.pop('d405',None)
    registry.store.save()
    observation=capture(registry,camera)
    ident=observation['observation_id']
    latest[camera]=ident
    registry.store.save()
    output=registry.store.path/'battery-observations'/call_id
    output.mkdir(parents=True,exist_ok=True)
    with gpu_slot():
        recognition=segment(registry.store,ident,output,lambda:registry.store.cancelled)
    location=None
    status='recognition_rejected'
    if recognition.get('status')=='perception_accepted':
        mask=np.asarray(Image.open(output/'mask.png').convert('L'))
        try:
            with np.load(frame_path(registry.store,ident),allow_pickle=False) as frame:
                location=surface_location(frame,mask)
            status='localized'
        except ValueError as error:
            status='depth_rejected'
            depth_error=str(error)
    elif any((recognition.get(key) or {}).get('status')=='not_found' for key in ('selection','coarse_selection')):
        status='target_not_found'
    if registry.store.cancelled:raise AgentError('cancelled','观察定位已取消')
    result=dict(observation_id=ident,camera=camera,status=status,location=location,
        recognition=recognition,message=depth_error if status=='depth_rejected' else '',calibration_sha256=observation.get('calibration_sha256'),
        image_time=observation['cameras'][camera]['captured_at'],executable=False)
    path=output/'result.json';atomic_json(path,result)
    relative=str(path.relative_to(registry.store.path))
    registry.store.state.setdefault('battery_observations',{})[ident]=relative
    registry.store.save()
    return {key:result[key] for key in ('observation_id','camera','status','location','image_time','message')} | {'result_file':relative}
