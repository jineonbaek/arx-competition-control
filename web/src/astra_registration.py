"""Astra calibration validation and depth-to-color projection; no device I/O."""
import ctypes
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np


class FactoryParams(ctypes.Structure):
    # Exact 120-byte OBCameraParams from the pinned Orbbec OniCTypes.h.
    _fields_=[('l_intr_p',ctypes.c_float*4),('r_intr_p',ctypes.c_float*4),
              ('r2l_r',ctypes.c_float*9),('r2l_t',ctypes.c_float*3),
              ('l_k',ctypes.c_float*5),('r_k',ctypes.c_float*5)]


def factory_parameters(device):
    if ctypes.sizeof(FactoryParams)!=120:raise RuntimeError('Unsafe OBCameraParams ABI')
    try:params=device.get_property(14,FactoryParams)
    except Exception as error:
        return dict(status="unavailable",source="Orbbec_OpenNI_property_14",reason=repr(error))
    values={name:list(getattr(params,name)) for name,_ in FactoryParams._fields_}
    flat=np.array([v for row in values.values() for v in row],dtype=float)
    rotation=np.asarray(values['r2l_r']).reshape(3,3)
    valid=bool(np.isfinite(flat).all()
               and all(v[0]>0 and v[1]>0 and 0<=v[2]<640 and 0<=v[3]<480
                       for v in [values['l_intr_p'],values['r_intr_p']])
               and np.allclose(rotation.T@rotation,np.eye(3),atol=1e-3)
               and np.isclose(np.linalg.det(rotation),1,atol=1e-3))
    report=dict(status='valid' if valid else 'invalid',source='Orbbec_OpenNI_property_14',sha256=hashlib.sha256(bytes(params)).hexdigest())
    if valid:report['parameters']=values
    else:report['reason']='Factory camera parameters are non-finite or invalid; RGB-D calibration required'
    return report


def intrinsics(matrix,distortion,source):
    return dict(fx=float(matrix[0,0]),fy=float(matrix[1,1]),cx=float(matrix[0,2]),cy=float(matrix[1,2]),
                distortion_model='opencv_brown_conrady',coeffs=np.asarray(distortion).tolist(),source=source)


def load_calibration(path,serial,size=(640,480)):
    payload=Path(path).read_bytes()
    data=json.loads(payload)
    if data.get('serial')!=serial:raise ValueError('Calibration belongs to a different Astra')
    if data.get('depth_size')!=list(size) or data.get('color_size')!=list(size):
        raise ValueError('Calibration resolution does not match the capture profile')
    for key in ['depth_K','color_K']:
        k=np.asarray(data[key],dtype=np.float64)
        if k.shape!=(3,3) or not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0:
            raise ValueError('Invalid '+key)
        if not np.allclose(k[2],[0,0,1]) or not 0<=k[0,2]<size[0] or not 0<=k[1,2]<size[1]:
            raise ValueError('Invalid camera matrix or principal point')
        data[key]=k
    for key in ['depth_D','color_D']:
        d=np.asarray(data[key],dtype=np.float64)
        if d.shape!=(5,) or not np.isfinite(d).all():raise ValueError('Invalid '+key)
        data[key]=d
    rotation=np.asarray(data['depth_to_color_R'],dtype=np.float64)
    translation=np.asarray(data['depth_to_color_translation_m'],dtype=np.float64)
    if (rotation.shape!=(3,3) or not np.isfinite(rotation).all() or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-4)
            or not np.isclose(np.linalg.det(rotation),1,atol=1e-4)):
        raise ValueError('Invalid depth-to-color rotation')
    if translation.shape!=(3,) or not np.isfinite(translation).all() or np.linalg.norm(translation)>.5:
        raise ValueError('Invalid translation in metres')
    data['depth_to_color_R']=rotation;data['depth_to_color_translation_m']=translation
    data['sha256']=hashlib.sha256(payload).hexdigest()
    return data


class Registration:
    def __init__(self,calibration):
        self.calibration=calibration
        self.k=calibration['depth_K'];self.size=tuple(calibration['color_size'])
        self.maps=cv2.initUndistortRectifyMap(self.k,calibration['depth_D'],None,self.k,tuple(calibration['depth_size']),cv2.CV_32FC1)
        self.transform=np.eye(4,dtype=np.float64)
        self.transform[:3,:3]=calibration['depth_to_color_R']
        self.transform[:3,3]=calibration['depth_to_color_translation_m']
        y,x=np.indices((calibration['depth_size'][1],calibration['depth_size'][0]))
        rotation=self.transform[:3,:3]
        self.z_factor=rotation[2,0]*(x-self.k[0,2])/self.k[0,0]+rotation[2,1]*(y-self.k[1,2])/self.k[1,1]+rotation[2,2]
        self.intrinsics=intrinsics(calibration['color_K'],calibration['color_D'],'measured_rgbd_calibration')

    def apply(self,raw):
        if raw.dtype!=np.uint16 or raw.shape!=self.z_factor.shape:raise ValueError('Invalid native depth frame')
        rectified=cv2.remap(raw,*self.maps,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
        metres=rectified.astype(np.float32)*.001
        invalid=(rectified==0) | (metres*self.z_factor+self.transform[2,3]<=0)
        metres[invalid]=np.nan
        registered=cv2.rgbd.registerDepth(self.k,self.calibration['color_K'],self.calibration['color_D'],self.transform,metres,self.size,depthDilation=False)
        valid=np.isfinite(registered) & (registered>0) & (registered<=65.535)
        result=np.zeros(registered.shape,np.uint16)
        result[valid]=np.rint(registered[valid]*1000).astype(np.uint16)
        return result
