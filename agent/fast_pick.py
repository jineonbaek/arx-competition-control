"""Standalone fast pickup, bound to one stopped docking receipt and wrist observation."""


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
