"""Finite RGB re-observation and Y-only search for the two placement entries."""
import time
import numpy as np


class FeatureUnavailable(ValueError):
    """A valid image lacks visible comparison edges (not a camera/pose fault)."""


class HorizontalRecovery:
    offsets_mm=(5.,-5.,10.,-10.,15.,-15.)
    max_observations=24
    observation_budget_s=45.

    def __init__(self):
        self.started=time.monotonic();self.attempts=0;self.probes=0
        self.misses=0;self.invalid=0;self.last_stamp=None;self.anchor=None
        self.message='D405 正在识别';self.mode='observe'

    def _stop(self,message):
        self.mode='exhausted';self.message=message+'；保持夹持'
        raise ValueError(self.message)

    def before_observe(self):
        # Budget never resets after a successful frame or a probe. A noisy
        # aligned/unaligned sequence must not create another infinite loop.
        if self.attempts>=self.max_observations or time.monotonic()-self.started>=self.observation_budget_s:
            self._stop('D405 对位观测预算已用完，尚未收敛')
        self.attempts+=1

    def failed(self,error,frame,tool,*,allow_search=True,after_image_time=None):
        stamp=float(frame.get('image_time',float('nan'))) if frame is not None else float('nan')
        fresh=(np.isfinite(stamp) and -.25<=time.time()-stamp<=2.
               and (self.last_stamp is None or stamp>self.last_stamp)
               and (after_image_time is None or stamp>after_image_time))
        if not allow_search or not isinstance(error,FeatureUnavailable) or not fresh:
            self.invalid+=1;self.misses=0;self.mode='retry'
            self.message='D405 有效画面重试 '+str(self.invalid)+'/3：'+str(error)
            if self.invalid>=3:self._stop('D405 连续三次无法获得有效配对画面：'+str(error))
            return None
        self.invalid=0;self.last_stamp=stamp;self.misses+=1
        self.mode='retry';self.message='D405 边沿不可见，准备水平探查：'+str(error)
        if self.misses<2:return None
        if self.probes>=len(self.offsets_mm):
            self._stop('D405 水平探查已完成，左右 15 mm 内仍未识别到仓口和电池边沿')
        tool=np.asarray(tool,float)
        if tool.shape!=(4,4) or not np.isfinite(tool).all():
            self._stop('D405 水平探查缺少有效末端位姿')
        if self.anchor is None:self.anchor=float(tool[1,3])
        offset=self.offsets_mm[self.probes];self.probes+=1;self.misses=0
        target=tool.copy();target[1,3]=self.anchor+offset/1000.
        self.mode='search'
        self.message=f'D405 水平探查 {self.probes}/6：相对探查起点 Y {offset:+.0f} mm'
        return target

    def recognized(self):
        self.misses=0;self.invalid=0;self.anchor=None
        self.mode='align';self.message='已识别两条边，按视觉偏差对齐'

    def snapshot(self):
        return dict(mode=self.mode,message=self.message,attempt=self.attempts,
                    probes=self.probes,max_probes=len(self.offsets_mm),search_radius_mm=15.,
                    max_observations=self.max_observations,observation_budget_s=self.observation_budget_s,
                    elapsed_s=max(0.,time.monotonic()-self.started))


def rgb_correction_step(measurement,remaining_mm):
    """RGB plane scale is approximate: damp and remeasure every small move.

    Apply this after accounting for motion since exposure. Depth-calibrated
    estimates keep their existing behavior; no minimum movement is imposed.
    """
    if measurement.get('measurement_source')!='rgb_only':return remaining_mm
    return float(np.clip(.5*remaining_mm,-5.,5.))
