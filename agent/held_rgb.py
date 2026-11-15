"""Conservative file-only D405 partial-reference tracker; reports image-plane evidence."""
import copy
import hashlib
import math
import cv2
import numpy as np
from .held_battery import validate_identity,finite


class ReferenceMatcher:
    def __init__(self,rgb,battery_mask,jaw_mask,*,identity,reference_kind,empty_rgb=None,
                 max_offset_px=7.,max_angle_deg=5.):
        if reference_kind not in ('synthetic','annotated_archive'):raise ValueError('A separately qualified offline reference is required; first-frame auto-reference is forbidden')
        self.identity=validate_identity(identity);self.kind=reference_kind
        self.rgb=np.array(rgb,copy=True);self.masks=[np.array(v,dtype=np.uint8,copy=True) for v in (battery_mask,jaw_mask)]
        if (self.rgb.dtype!=np.uint8 or self.rgb.ndim!=3 or self.rgb.shape[2]!=3
                or any(m.shape!=self.rgb.shape[:2] or np.count_nonzero(m)<100 for m in self.masks)
                or np.any((self.masks[0]>0)&(self.masks[1]>0))):raise ValueError('Nonoverlapping local battery/jaw regions required')
        if not finite(max_offset_px) or not finite(max_angle_deg) or min(max_offset_px,max_angle_deg)<=0:raise ValueError('Invalid image-plane alignment limits')
        self.max_offset=max_offset_px;self.max_angle=max_angle_deg
        self.gray=cv2.cvtColor(self.rgb,cv2.COLOR_RGB2GRAY)
        self.points=[cv2.goodFeaturesToTrack(self.gray,100,.02,3,mask=m) for m in self.masks]
        if any(p is None or len(p)<8 for p in self.points):raise ValueError('Insufficient discriminative battery or jaw features')
        self.empty=None if empty_rgb is None else np.array(empty_rgb,copy=True)
        if self.empty is not None and (self.empty.dtype!=np.uint8 or self.empty.shape!=self.rgb.shape):raise ValueError('Empty reference shape mismatch')
        self.empty_gray=None if self.empty is None else cv2.cvtColor(self.empty,cv2.COLOR_RGB2GRAY)
        self.digest=hashlib.sha256(self.rgb.tobytes()+b''.join(m.tobytes() for m in self.masks)+(b'' if self.empty is None else self.empty.tobytes())).hexdigest()

    def _fit(self,current,index):
        source=self.points[index];target,ok,_=cv2.calcOpticalFlowPyrLK(self.gray,current,source,None,winSize=(21,21),maxLevel=3)
        if target is None:return None
        back,back_ok,_=cv2.calcOpticalFlowPyrLK(current,self.gray,target,None,winSize=(21,21),maxLevel=3)
        valid=(ok.ravel()>0)&(back_ok.ravel()>0)&(np.linalg.norm(source-back,axis=2).ravel()<1.)
        valid&=np.isfinite(target).all(axis=(1,2))
        if valid.sum()<8:return None
        a=source[valid].reshape(-1,2);b=target[valid].reshape(-1,2)
        matrix,inliers=cv2.estimateAffinePartial2D(a,b,method=cv2.RANSAC,ransacReprojThreshold=1.5)
        if matrix is None or inliers.sum()<8 or inliers.mean()<.7:return None
        scale=np.linalg.norm(matrix[:,0])
        if not .8<scale<1.2:return None
        aligned=cv2.warpAffine(current,matrix,(current.shape[1],current.shape[0]),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP)
        mask=self.masks[index]>0;x=self.gray[mask].astype(float);y=aligned[mask].astype(float)
        correlation=float(np.corrcoef(x,y)[0,1]) if min(x.std(),y.std())>3 else 0.
        if not math.isfinite(correlation) or correlation<.83:return None
        residual=np.linalg.norm(a@matrix[:,:2].T+matrix[:,2]-b,axis=1)[inliers.ravel()>0]
        full=np.eye(3);full[:2]=matrix
        return full,float(max(.25,np.percentile(residual,90))),float(correlation)

    def analyze(self,rgb,*,frame_id,captured_at):
        r=dict(mode='offline',executable=False,identity=copy.deepcopy(self.identity),frame_id=frame_id,captured_at=captured_at,
            presence='unknown',engagement='unknown',pose='unobservable',critical_visible=False,
            image_delta=None,uncertainty_px=None,metric_pose=None,
            observable_axes=[],reference_kind=self.kind,reference_sha256=self.digest,reason='Critical reference features not observed')
        image=np.asarray(rgb)
        if image.dtype!=np.uint8 or image.shape!=self.rgb.shape or not isinstance(frame_id,str) or not frame_id or not finite(captured_at):return r
        gray=cv2.cvtColor(image,cv2.COLOR_RGB2GRAY);jaw=self._fit(gray,1)
        if jaw is None:return r
        battery=self._fit(gray,0)
        if battery is None:
            if self.empty_gray is not None:
                aligned=cv2.warpAffine(gray,jaw[0][:2],(gray.shape[1],gray.shape[0]),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP)
                difference=np.mean(abs(aligned[self.masks[0]>0].astype(float)-self.empty_gray[self.masks[0]>0]))
                if difference<6.:
                    r.update(presence='absent',engagement='visibly_incorrect',critical_visible=True,reason='Local region matches separate empty reference')
            return r
        relative=np.linalg.inv(jaw[0])@battery[0];yy,xx=np.nonzero(self.masks[0]);center=np.array([xx.mean(),yy.mean(),1.])
        displacement=(relative@center-center)[:2];angle=float(np.degrees(np.arctan2(relative[1,0],relative[0,0])))
        good=np.linalg.norm(displacement)<=self.max_offset and abs(angle)<=self.max_angle and abs(np.linalg.norm(relative[:2,0])-1)<.03
        r.update(presence='present',engagement='visually_consistent' if good else 'visibly_incorrect',
            pose='matched' if good else 'misaligned',critical_visible=True,image_delta=[*displacement.tolist(),angle],
            uncertainty_px=battery[1]+jaw[1],observable_axes=['image_x','image_y','image_rotation'],
            reason='Observed image-plane grip relationship; depth, 6DoF and force remain unverified',
            battery_affine=battery[0][:2].tolist(),jaw_affine=jaw[0][:2].tolist())
        return r
