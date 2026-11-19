# Lift accuracy and speed candidate — 2026-11-19

## Scope and evidence

The operator reports inaccurate stopping from both manual height commands and
automatic pre-lift, plus slow travel. Both use `/lift_controller` and its
`fixed_body_height` parameter; heights are SDK motor radians, not millimetres.

Read-only inspection on 2026-11-19 found a target of `0.0` and stable feedback
near `0.136 SDK rad`. Historical bidirectional runs also show persistent,
direction-dependent endpoint error. The loaded vendor controller uses an outer
position P gain of 2 and gravity feedforward of -2, without integral correction.
These observations do not establish a mechanical fault or safe replacement gains.

The web workflow separately floors fractional acceptance heights and the site
policy accepts 2.65 for a target of 3.0. That can stop an automatic lift early.

## Candidate behavior

- Keep the exact requested web height in both target and acceptance fields.
- Normalize web workflow acceptance to its selected target, including legacy
  saved settings and policies. Do not mutate saved runs or policy files on disk.
- An acceptance equal to the target uses two-sided position tolerance and the
  complete configured settling window. Keep deliberately lower acceptance
  available to standalone coarse-alignment callers for compatibility.
- Use body feedback's own timestamp for lift settling, and require new body
  feedback after stop. Recheck the exact endpoint during gripper opening and
  its submission guard, through the observer handoff. Later observer movement
  retains the existing drift contract.
- Provide a startup-only ROS parameter, `lift_target_ramp_rad_s`, default 0.5,
  bounded to `(0, 1.0] SDK rad/s`. The vendor field is an increment per 400 Hz
  tick, so divide the requested rate by 400. Homing remains 0.5 rad/s.
- Reject invalid rates before SDK construction and reject changes while running.
  Keep motor mode, position gain, gravity compensation, torque limit, chassis
  channels, CAN ownership, stop service and calibration behavior unchanged.

The 1.0 ceiling is a candidate software bound, not a verified physical speed or
mechanical safety rating. Changing the reference rate does not correct steady
position error. Strict arrival may now expose that error as a stall instead of
allowing the grasp to continue at the wrong height.

## Modules and data flow

`web/src/battery_task.py` validates the browser height and stores exact task
settings. `agent/battery_production.py` constructs an exact-height web policy.
`agent/coarse_execution.py` consumes measured feedback through `ArrivalMonitor`.
`web/hardware/slow_lift_config.hpp` validates and converts ramp units;
`lift_controller_web.cpp` declares the read-only parameter before constructing
the existing SDK controller. No database/schema or browser API change is needed.

## Build, operation and remaining validation

This candidate is developed in `/home/arx/arx-lift-accuracy-speed-20261119`,
separate from the running checkout and protected `/home/arx/LIFT`. Restore vendor
SDKs and ROS message packages using the existing repository setup instructions.
Offline tests reuse the site's Python environment and ignored, read-only links
to existing CAD/browser assets. Do not commit generated assets, logs or binaries.

Build only into a separate build directory; do not install or run the driver as
an offline test. There is no cloud deployment for this physical controller.
After an agreed safe handoff at a supported low position, an operator can select
a reviewed rate with `--ros-args -p lift_target_ramp_rad_s:=<rate>`; omission keeps
0.5. Restarting the body driver re-homes and also affects the shared chassis.
Never start a second CAN5 controller or restart a loaded/raised platform.

Before tuning or enabling higher speed, collect a bounded low-position up/down
baseline through the existing controller with an onsite observer and available
emergency stop. Compare target/reference/feedback and actual measured height,
check stopping and hold behavior under the current load, then determine a
control correction from measured data. No gain or firmware changes are included
in this candidate. Physical accuracy and improved travel speed remain unverified.

## Validation

44 focused Python regressions and five pure C++ checks pass. The native lift
driver compiled without execution. The final complete Agent/web suite has 733
passes and 25 failures; all 25 reproduce unchanged on base ff14e6b, with no new
failure names. Full failure names and test boundaries are recorded in
[lift candidate validation](lift-accuracy-speed-validation.md).

A later read-only sample found target 15.09 and stable feedback around 15.1726
SDK rad, following existing webpage commands. No diagnostic movement was sent.
This is further software-feedback evidence of endpoint error, not a measured
physical-height calibration.
