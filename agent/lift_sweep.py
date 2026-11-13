"""Offline fixed-joint lift sweep against the CAD column; never motion permission."""
import math
import numpy as np
from web.src.handeye_column import ColumnModel, NOMINAL_RAD_PER_M


def column_sweep(model, q, start, end):
    travel = abs(end-start)/NOMINAL_RAD_PER_M
    steps = max(1, math.ceil(travel/.005))
    while True:
        gap, link = min(model.pose(q, h)[0] for h in np.linspace(start, end, steps+1))
        # Refine only unresolved CAD intervals, without a fixed clearance margin.
        lower = gap-travel/(2*steps)
        if lower>0 or gap<=0 or travel/(2*steps)<1e-8:break
        steps*=2
    return dict(clear=bool(lower>0), lower_mm=round(max(0.,lower)*1000,1),
                margin_mm=0, samples=steps+1, link=link)


def assess_lift(frame, target):
    result = dict(status='unavailable', both_arms_column_checked=False,
                  environment_checked=False, executable=False, arms={})
    try:
        right = np.asarray(frame['joints'],float)
        left = np.asarray(frame['left_joints'],float)
        start = float(frame['lift_rad'])
        stamps = np.array([float(frame[k]) for k in
                           ('image_time','arm_time','left_arm_time','body_time')])
        if (right.shape!=(6,) or left.shape!=(6,) or
                not np.isfinite(np.r_[right,left,start,target,stamps]).all() or
                not 0<=start<=18 or not 0<=target<=18 or
                np.max(np.abs(stamps[1:]-stamps[0]))>.2):
            raise ValueError('双臂、升降反馈无效、未配对，或目标超出 0～18 rad')
    except (KeyError, TypeError, ValueError) as error:
        result['message'] = '无法检查双臂升降立柱间隙：'+str(error)
        return result
    arms = {side:column_sweep(ColumnModel(side=side),q,start,target)
            for side,q in (('left',left),('right',right))}
    clear = all(arm['clear'] for arm in arms.values())
    return result | dict(status='column_clear' if clear else 'column_rejected',
        arms=arms, both_arms_column_checked=True,
        message=('双臂固定当前关节时，升降沿途立柱间隙通过；仍未检查周围环境' if clear
                 else '升降沿途双臂与立柱间隙不足；不可执行该建议'))
