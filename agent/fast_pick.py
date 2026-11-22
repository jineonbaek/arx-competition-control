"""Standalone fast pickup, bound to one stopped docking receipt and wrist observation."""

import json
import math
from pathlib import Path
from numbers import Real

FAST_PREGRASP_PATH=Path(__file__).resolve().parents[1]/"config/fast-pregrasp.local.json"

FAST_PREGRASP_JOINTS_DEG=(19.66914334204537,29.2725054007236,43.37099254278322,
                         -11.494709293372578,19.89204159249508,-88.55883048579047)


def pregrasp_joints(value):
    if (not isinstance(value,list) or len(value)!=6
            or any(isinstance(v,bool) or not isinstance(v,Real) or not math.isfinite(v) for v in value)):
        raise ValueError('快模式预抓点须为六轴有限角度')
    return [float(v) for v in value]


def load_pregrasp():
    try:return pregrasp_joints(json.loads(FAST_PREGRASP_PATH.read_text(encoding='utf-8')))
    except FileNotFoundError:return list(FAST_PREGRASP_JOINTS_DEG)


def prepare_pregrasp(flow):
    """Fast entry only: reuse the joint move and existing taught-pose refinement."""
    import time
    import numpy as np
    recovery=flow.recovery;store=flow.store
    origin=recovery.capture()['joints_rad']
    target=load_pregrasp()
    store.event('fast_pregrasp',dict(stage='前往快模式预抓取点',target_joints_deg=target))
    try:
        trace=recovery._move([origin,np.radians(target).tolist()])
    except Exception:
        recovery.controller.begin_contact_stop(recovery.token)
        recovery.controller.interrupt_motion(recovery.token)
        try:recovery.controller.submit({'type':'arm.stop','arm':'right'},recovery.token)
        except Exception:pass
        raise
    completed_at=time.time()
    recovery.state.update(outbound_trace=trace,last_approach_end=trace[-1])
    store.state['fast_pregrasp']=dict(target_joints_deg=target,completed_at=completed_at)
    enter_pregrasp_refinement(flow,completed_at)
    store.event('fast_pregrasp',dict(stage='预抓取点轨迹完成，进入 D405 精调'))


def enter_pregrasp_refinement(flow,completed_at):
    store=flow.store
    store.state['mission'].update(stage='COARSE_ALIGNED',point3_refinement=True,
        evidence_barriers={'PICK_AT_PREGRASP':dict(completed_at=completed_at,
            previous_observation_ids=list(store.state.get('observations',{})))})
    flow._next('wrist')


def fast_requested(store):
    return store.state.get('battery_task_settings', {}).get('pickup_flow') == 'fast'


def stopped_for_fast_pick(store):
    dock = store.state.get('fast_dock') or {}
    return (fast_requested(store) and dock.get('run_id') == store.state['run_id']
            and dock.get('status') == 'stopped' and dock.get('source') == 't1_short_pulse'
            and isinstance(dock.get('stopped_at'), (int, float)) and dock['stopped_at'] > 0)


def direct_plan(plan):
    return (plan.get('pickup_flow') == 'fast' and plan.get('vision_mode') == 'd405'
            and bool(plan.get('snapshot_id')) and bool(plan.get('observation_id')))


def handle_support(frame, mask, pose, profile):
    """Count measured foreground samples on both halves of the CAD pull bridge."""
    import numpy as np
    from .grasp_geometry import rigid
    transform = rigid(pose['T_camera_from_object'])
    xyz = frame['xyz']
    selected = (mask > 0) & np.isfinite(xyz).all(axis=2) & (xyz[..., 2] > .07)
    obj = ((xyz[selected] - transform[:3, 3]) @ transform[:3, :3]) * 1000
    front = profile['points_object_mm']['front']
    lo, hi = profile['hole_bounds_mm']
    bridge = ((obj[:, 2] >= hi[2]) & (obj[:, 2] <= front[2] + 3.)
              & (np.abs(obj[:, 1] - front[1]) <= profile['handle_thickness_mm'] / 2 + 3.))
    left = int((bridge & (obj[:, 0] >= lo[0]) & (obj[:, 0] < front[0] - 2.)).sum())
    right = int((bridge & (obj[:, 0] <= hi[0]) & (obj[:, 0] > front[0] + 2.)).sum())
    return dict(confirmed=min(left, right) >= 6, left_pixels=left, right_pixels=right,
                snapshot_id=str(frame['id']))
