"""Restrict a seeded battery mask to depth points near the observed CAD volume."""
import numpy as np

def validate_seeded_pose(seed,estimate):
    seed=np.asarray(seed,float);estimate=np.asarray(estimate,float)
    if seed.shape!=(4,4) or estimate.shape!=(4,4) or not np.isfinite(np.r_[seed.ravel(),estimate.ravel()]).all():
        raise ValueError('Invalid seeded pose')
    for matrix in (seed,estimate):
        R=matrix[:3,:3]
        if not np.allclose(R.T@R,np.eye(3),atol=1e-3) or np.linalg.det(R)<.999:
            raise ValueError('Invalid rotation')
    angle=np.degrees(np.arccos(np.clip((np.trace(seed[:3,:3].T@estimate[:3,:3])-1)/2,-1,1)))
    distance=np.linalg.norm(seed[:3,3]-estimate[:3,3])*1000
    if angle>15. or distance>10.:
        raise ValueError('Seeded body pose disagrees with previous held object')
    return dict(rotation_difference_deg=float(angle),translation_difference_mm=float(distance))

def filter_seeded_mask(mask,xyz,base_from_camera,base_from_object,bounds_m,*,padding_m=.012):
    mask=np.asarray(mask)>0;xyz=np.asarray(xyz,float);cam=np.asarray(base_from_camera,float)
    obj=np.asarray(base_from_object,float);bounds=np.asarray(bounds_m,float)
    if (xyz.shape!=mask.shape+(3,) or cam.shape!=(4,4) or obj.shape!=(4,4) or bounds.shape!=(2,3)
            or not np.isfinite(np.r_[cam.ravel(),obj.ravel(),bounds.ravel(),padding_m]).all()
            or np.any(bounds[1]<=bounds[0]) or not 0<=padding_m<=.02):
        raise ValueError('Invalid seeded depth mask inputs')
    good=mask & np.isfinite(xyz).all(axis=2) & (xyz[:,:,2]>.1) & (xyz[:,:,2]<1.5)
    base=xyz@cam[:3,:3].T+cam[:3,3]
    local=(base-obj[:3,3])@obj[:3,:3]
    selected=good & np.all((local>=bounds[0]-padding_m)&(local<=bounds[1]+padding_m),axis=2)
    if selected.sum()<250 or selected.sum()<.5*good.sum():
        raise ValueError('Seeded mask inconsistent with observed battery depth')
    return selected.astype(np.uint8)*255
