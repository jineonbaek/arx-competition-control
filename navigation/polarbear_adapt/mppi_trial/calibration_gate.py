"""Wait for measurement and both frame transforms before an attended pulse.

Readiness only: never publish a command, relax freshness, or ignore occupancy.
"""
import time
from tf2_ros import TransformException

def wait_ready(node,timeout=12.):
    deadline=time.monotonic()+timeout
    reason='no samples'
    while time.monotonic()<deadline:
        node.spin(.04)
        stale=node.stale_inputs()
        if stale:
            reason='stale inputs: '+','.join(stale)
            continue
        try:
            for frame in ('map','odom'):node.pose(frame,node.now())
        except (TransformException,ValueError) as exc:
            reason=str(exc)
            continue
        return
    raise RuntimeError('Calibration discovery/freshness timeout: '+reason)
