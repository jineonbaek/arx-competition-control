"""D435 RGB-D mask for the held battery, excluding the colored gripper."""
import cv2
import numpy as np


def held_battery_mask(rgb,xyz,base_from_color,base_from_object,bounds_mm,*,
                      projected_center_px,min_area=250,margin_mm=10.,
                      saturation_max=50,value_max=160):
    rgb=np.asarray(rgb);xyz=np.asarray(xyz,float)
    camera=np.asarray(base_from_color,float);obj=np.asarray(base_from_object,float)
    bounds=np.asarray(bounds_mm,float)/1000.;center=np.asarray(projected_center_px,float)
    if (rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3 or xyz.shape!=rgb.shape
            or camera.shape!=(4,4) or obj.shape!=(4,4) or bounds.shape!=(2,3)
            or center.shape!=(2,) or not np.isfinite(np.r_[camera.ravel(),obj.ravel(),bounds.ravel(),center]).all()):
        raise ValueError('Invalid D435 held-battery mask input')
    base=xyz@camera[:3,:3].T+camera[:3,3]
    local=(base-obj[:3,3])@obj[:3,:3]
    margin=float(margin_mm)/1000.
    inside=(np.isfinite(xyz).all(2)&(xyz[:,:,2]>.05)
            &(local>=bounds[0]-margin).all(2)&(local<=bounds[1]+margin).all(2))
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    candidate=(inside&(hsv[:,:,1]<=saturation_max)&(hsv[:,:,2]<=value_max)).astype(np.uint8)
    count,labels,stats,centers=cv2.connectedComponentsWithStats(candidate,8)
    choices=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=min_area]
    if not choices:raise ValueError('No D435 battery body inside expected RGB-D volume')
    chosen=min(choices,key=lambda i:np.linalg.norm(centers[i]-center))
    mask=(labels==chosen).astype(np.uint8)*255
    ys,xs=np.nonzero(mask)
    return mask,[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)]
