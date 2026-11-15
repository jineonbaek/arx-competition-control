"""Non-motion installation diagnostics. There is no provider or actuation route."""
import inspect

from .cabinet_controller_bridge import BridgePolicy, BridgeRejected, ControllerBridge, digest


def readiness(controller):
    supported = 'expected_driver' in inspect.signature(controller.start_trajectory).parameters
    return dict(bridge_installed=True, driver_binding_supported=supported,
                physical_provider_configured=False, execution_enabled=False,
                reason=('Validated physical provider missing' if supported else
                        'Controller requires approved driver-epoch dispatch contract'),
                probe_available=True, probe_sends_motion=False)


def admission_probe(controller, data):
    """Exercise fail-closed admission, with fixed measured start and no provider.

    Browser input cannot select a plan, provider, approval, target or enable flag.
    The bridge must reject before it reserves ownership or dispatches anything.
    """
    if type(data) is not dict or data != {'op': 'admission_probe'}:
        raise ValueError('Only the fixed admission_probe operation is accepted')
    with controller.lock:
        state = controller.snapshot()
        if (state.get('motion_reserved') is not False
                or state.get('competition_reserved', False) is not False
                or state.get('chassis_active') is not False):
            raise ValueError('Admission probe requires idle motion and competition ownership')
        epoch, owner = controller.command_epoch, controller.motion_owner
        try:
            # Diagnostic policy only. No production physical provider is supplied.
            bridge = ControllerBridge(controller, BridgePolicy(.002, .02, .02, .02,
                                                               .3, .2, .2, 8.))
            if not state.get('arms', {}).get('right', {}).get('fresh'):
                raise BridgeRejected('Fresh right-arm feedback required for diagnostic')
            q = list(state['arms']['right']['joints'])
            plan = dict(interpolation='segment_quintic_scalar', time_from_start_s=[0., 1.],
                        joint_waypoints_rad=[q, q.copy()])
            binding = dict(run_id='readiness-probe', boot_id='diagnostic-no-motion', generation=0,
                           asset_hash=digest(controller.config), held_identity_hash='0'*64)
            bridge.start('outside_align', plan, binding)
        except BridgeRejected as error:
            if controller.command_epoch != epoch or controller.motion_owner is not owner:
                raise RuntimeError('Admission diagnostic unexpectedly changed ownership/epoch') from error
            return dict(readiness(controller), blocked_before_reservation=True,
                        motion_commands_sent=False, reason=str(error))
        raise RuntimeError('Admission diagnostic unexpectedly passed without physical provider')
