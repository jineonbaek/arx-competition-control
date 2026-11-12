"""Explicit local assembly; importing this module never reserves or moves hardware."""
from .battery_backend import BatteryControllerBackend
from .battery_flow import BatteryFlow
from .battery_tcp import load_pick_setup
from .battery_tools import BatteryExecutionTools
from .perception_session import PerceptionSession


def build_workflow(config,store,controller,token,*,policies,refinement_policy,
                   timing_limits,alignment_tolerance_deg,j6_tolerance_deg,
                   preflight,review_plan,timeout_s,observations=None,
                   clearance_mm=50.):
    setup=load_pick_setup(timing_limits=timing_limits,alignment_tolerance_deg=alignment_tolerance_deg,
                          j6_tolerance_deg=j6_tolerance_deg)
    backend=BatteryControllerBackend(controller,token,store,policies=policies,pick_setup=setup,
        preflight=preflight,cancelled=lambda:store.cancelled,timeout_s=timeout_s)
    tools=BatteryExecutionTools(config,store,observations=observations,backends=backend,
                               pick_setup=setup,refinement_policy=refinement_policy)
    store.perception_session=PerceptionSession(store.path)
    return BatteryFlow(tools,review_plan=review_plan,clearance_mm=clearance_mm,
                       receipt_timeout_s=max(timeout_s.values())+5.)
