"""Relative D405 feature tracking; no force/full-6D/metric-pose claim."""
import cv2
import numpy as np


class GripTracker:
    def __init__(self,*,max_age_s=.8,max_slip_px=6.,max_rotation_deg=4.,max_raw_delta=.05):
        values=[max_age_s,max_slip_px,max_rotation_deg,max_raw_delta]
        if not np.isfinite(values).all() or min(values)<=0:raise ValueError('Invalid grip tracking policy')
        self.max_age,self.max_slip,self.max_angle,self.max_raw=values
        self.reference=None;self.history=[];self.latched=False;self.last_time=-np.inf

    @staticmethod
    def _shape(rgb):
        rgb=np.asarray(rgb)
        if rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3:raise ValueError('RGB image required')
        h,w=rgb.shape[:2]
        mask=(cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)[:,:,2]<100).astype(np.uint8)
        mask[:int(.12*h)]=0;mask[int(.88*h):]=0
        mask[:,:int(.18*w)]=0;mask[:,int(.82*w):]=0
        count,labels,stats,centers=cv2.connectedComponentsWithStats(mask,8)
        candidates=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>.035*h*w]
        if not candidates:raise ValueError('Held battery silhouette unavailable')
        index=min(candidates,key=lambda i:np.linalg.norm((centers[i]-[w/2,h*.45])/[w,h]))
        return dict(center=centers[index],mask=(labels==index).astype(np.uint8)*255,
                    area=int(stats[index,cv2.CC_STAT_AREA]),shape=(h,w))

    def bind(self,rgb,stamp,raw,*,now):
        if self.reference is not None:raise ValueError('Create a new tracker to explicitly bind a new grasp')
        if not np.isfinite([stamp,raw,now]).all() or not 0<=now-stamp<=self.max_age:raise ValueError('Fresh grasp reference required')
        shape=self._shape(rgb);gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
        points=cv2.goodFeaturesToTrack(gray,100,.01,8,mask=shape['mask'])
        if points is None or len(points)<3:raise ValueError('Visible battery texture insufficient for relative tracking')
        self.reference=shape|dict(raw=float(raw),stamp=float(stamp),gray=gray,points=points)
        # Follow the SAME feature identities across adjacent exposures while
        # measuring their cumulative transform from the original grasp pixels.
        # Comparing every exposure directly with the first image aliases the
        # repeated print texture after a large lighting/background change.
        self.previous_gray=gray;self.previous_points=points.copy();self.original_points=points.copy()
        self.history=[float(stamp)];self.last_time=float(stamp)

    def update(self,rgb,stamp,raw,*,now):
        result=dict(stable=False,status='unknown',metric_pose=None,force_verified=False)
        if self.reference is None:return result|dict(reason='No bound held-object reference')
        if self.latched:return result|dict(status='slip_suspected',reason='Relative change latched; no automatic rebaseline')
        if not np.isfinite([stamp,raw,now]).all() or not 0<=now-stamp<=self.max_age or stamp<self.last_time:
            self.history=[];return result|dict(reason='Stale/backward D405 evidence')
        try:shape=self._shape(rgb)
        except ValueError as error:
            self.history=[];return result|dict(reason=str(error))
        ref=self.reference
        if shape['shape']!=ref['shape']:
            self.history=[];return result|dict(reason='Camera resolution changed')
        area=shape['area']/ref['area'];raw_delta=abs(raw-ref['raw'])
        gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
        current,ok,_=cv2.calcOpticalFlowPyrLK(self.previous_gray,gray,self.previous_points,None,winSize=(21,21),maxLevel=3)
        if current is None:return result|dict(reason='Relative feature tracking unavailable')
        back,back_ok,_=cv2.calcOpticalFlowPyrLK(gray,self.previous_gray,current,None,winSize=(21,21),maxLevel=3)
        if back is None:return result|dict(reason='Relative feature consistency unavailable')
        valid=ok.ravel().astype(bool)&back_ok.ravel().astype(bool)&(np.linalg.norm(back-self.previous_points,axis=2).ravel()<1.)
        if valid.sum()<3:return result|dict(reason='Too few consistent held-object features')
        matrix,inliers=cv2.estimateAffinePartial2D(self.original_points[valid],current[valid],method=cv2.RANSAC,ransacReprojThreshold=2.)
        if matrix is None or not np.isfinite(matrix).all() or inliers is None or inliers.sum()<3 or inliers.mean()<.6:
            return result|dict(reason='Relative pose fit is ambiguous')
        shift=float(np.linalg.norm(matrix@np.r_[ref['center'],1.]-ref['center']))
        angle=abs(float(np.degrees(np.arctan2(matrix[1,0],matrix[0,0]))))
        scale=float(np.linalg.norm(matrix[:,0]))
        if shift>self.max_slip or angle>self.max_angle or raw_delta>self.max_raw or not .9<=scale<=1.1:
            self.latched=True
            return result|dict(status='slip_suspected',shift_px=shift,rotation_deg=angle,raw_delta=raw_delta)
        if not .6<=area<=1.4:
            self.history=[];return result|dict(reason='Held-object silhouette occluded or ambiguous')
        if stamp>self.last_time:
            # No feature replenishment or image rebaseline: new correspondences
            # retain their original coordinates, so gradual slip accumulates.
            keep=inliers.ravel().astype(bool)
            self.original_points=self.original_points[valid][keep]
            self.previous_points=current[valid][keep];self.previous_gray=gray
            self.history.append(float(stamp));self.last_time=float(stamp)
        self.history=[t for t in self.history if stamp-t<=self.max_age]
        stable=len(self.history)>=3 and self.history[-1]-self.history[0]>=.2
        return result|dict(stable=stable,status='stable_visible' if stable else 'settling',
                           shift_px=shift,rotation_deg=angle,raw_delta=raw_delta,
                           note='Relative image tracking only; does not certify grip force or hidden axes')
