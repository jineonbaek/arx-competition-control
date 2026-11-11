"""Bounded, offline D405 viewpoint proposals. No TCP or motion transport."""
from itertools import product
import numpy as np
from scipy.spatial.transform import Rotation
from web.src.camera_geometry import camera_intrinsics
from web.src.handeye_column import ColumnModel
from .grasp_geometry import rigid


def view_check(base_camera, bounds_m, k):
    transform=rigid(base_camera)
    bounds=np.asarray(bounds_m,float)
    if bounds.shape!=(2,3) or not np.isfinite(bounds).all() or np.any(bounds[0]>bounds[1]):
        raise ValueError('目标表面范围无效')
    values=[k[key] for key in ('width','height','fx','fy','cx','cy')]+list(k['coeffs'])
    if (len(k['coeffs'])!=5 or not np.isfinite(values).all() or min(k['fx'],k['fy'])<=0
            or min(k['width'],k['height'])<=48):
        raise ValueError('D405 内参无效')
    corners=np.array(list(product(*zip(bounds[0],bounds[1]))))
    camera=(corners-transform[:3,3])@transform[:3,:3]
    result=dict(in_view=False,depth_range_m=[float(camera[:,2].min()),float(camera[:,2].max())],
                edge_margin_px=None,pixel_bounds=None)
    if np.any(camera[:,2]<=.07) or np.any(camera[:,2]>=1.5):return result
    rs,intr=camera_intrinsics(k)
    pixels=np.array([rs.rs2_project_point_to_pixel(intr,p.tolist()) for p in camera])
    if not np.isfinite(pixels).all():return result
    lo,hi=pixels.min(0),pixels.max(0)
    margin=float(min(*lo,k['width']-1-hi[0],k['height']-1-hi[1]))
    return result|dict(in_view=margin>=24,edge_margin_px=margin,pixel_bounds=[lo.tolist(),hi.tolist()])


def assess_observer(frame, location, lift_rad, calibration, limits_deg):
    result=dict(status='observer_unavailable',executable=False,tcp=None,joints_rad=None,
                occlusion_checked=False,environment_checked=False,
                message='尚未找到 D405 观察姿态')
    try:
        q=np.asarray(frame['joints'],float)
        stamps=np.array([float(frame['image_time']),float(frame['arm_time'])])
        bounds=np.asarray(location['surface_bounds_p90_base_mm'],float)/1000
        target=np.asarray(location['surface_base_mm'],float)/1000
        limits=np.radians(np.asarray(limits_deg,float))
        if (q.shape!=(6,) or not np.isfinite(q).all() or not np.isfinite(stamps).all()
                or abs(stamps[0]-stamps[1])>.2 or not np.isfinite(lift_rad) or not 0<=lift_rad<=18):
            raise ValueError('缺少成对右臂反馈或升降目标无效')
        if (limits.shape!=(6,2) or not np.isfinite(limits).all() or np.any(limits[:,0]>=limits[:,1])
                or np.any(q<limits[:,0]) or np.any(q>limits[:,1]) or q[0]<np.radians(-35.8)):
            raise ValueError('右臂起姿或关节范围无效')
        if (target.shape!=(3,) or bounds.shape!=(2,3) or not np.isfinite(target).all()
                or not np.isfinite(bounds).all() or np.any(bounds[0]>target) or np.any(target>bounds[1])):
            raise ValueError('缺少有效的 D435 表面范围')
        camera=calibration['camera'];k=camera['intrinsics']
        if (calibration['parent_frame']!='right_link26' or camera['model']!='D405'
                or calibration['child_frame']!='D405 color optical frame'
                or not camera['serial'] or camera['serial']!=k['serial']):
            raise ValueError('D405 手眼标定坐标系或相机身份不匹配')
        handeye=rigid(calibration['T_right_link26_from_D405_color'])
        model=ColumnModel()

        def candidate(angles):
            wrist=model.wrist_transform(angles,lift_rad)
            base_camera=wrist@handeye
            view=view_check(base_camera,bounds,k)
            if not view['in_view']:return None
            path=model.check(q,angles,lift_rad)
            if not path['clear']:return None
            return result|dict(status='observer_candidate',joints_rad=np.asarray(angles).tolist(),
                lift_rad=float(lift_rad),wrist_base_mm=(wrist[:3,3]*1000).tolist(),
                wrist_rpy_deg=Rotation.from_matrix(wrist[:3,:3]).as_euler('xyz',degrees=True).tolist(),
                base_from_camera=base_camera.tolist(),view=view,column_path=path,
                arm_move_required=bool(np.max(np.abs(q-angles))>1e-6),
                surface_bounds_p90_base_mm=(bounds*1000).tolist(),
                intrinsics_source='D405 handeye calibration file',camera_serial=camera['serial'],
                message='有限候选中找到逆解和立柱路径检查通过的观察姿态；未执行。',
                note='预测视野仅覆盖 D435 已见表面的中央 90% 范围，非完整电池。未检查遮挡、环境和自碰撞；升降及移动后必须重新确认同一目标。')

        current=candidate(q)
        if current:
            return current|dict(stationary_plan=dict(version=1,kind='stationary',joint_position_rad=q.tolist()))
        orientations=[Rotation.from_matrix(model.wrist_transform(q,lift_rad)[:3,:3]).as_euler('xyz',degrees=True),
                      np.array([0.,0.,0.]),np.array([-90.,0.,0.])]
        best=None;cost=float('inf')
        # shortcut: finite local viewpoint search; use a scene planner if these candidates are insufficient.
        for rpy in orientations:
            rotation=Rotation.from_euler('xyz',rpy,degrees=True).as_matrix()
            for distance in (.30,.35,.40,.25,.45):
                wrist_pos=target-rotation@handeye[:3,2]*distance-rotation@handeye[:3,3]
                try:
                    angles=np.asarray(model.inverse_wrist(q,lift_rad,wrist_pos*1000,rpy,limits_deg))
                except ValueError:
                    continue
                item=candidate(angles)
                travel=float(np.linalg.norm(angles-q))
                if item is not None and travel<cost:best,cost=item,travel
        if best:return best
        result['message']='有限候选中未找到同时满足逆解、立柱间隙和预测视野的 D405 观察姿态'
    except (KeyError,ValueError,TypeError,AttributeError) as error:
        result['message']='观察姿态无法计算：'+str(error)
    return result
