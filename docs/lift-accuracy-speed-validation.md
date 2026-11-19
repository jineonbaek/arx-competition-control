# Lift candidate software validation — 2026-11-19

No real driver, motor constructor, CAN command, motion command or service
restart was executed by these checks. Native driver compilation is not hardware
validation. The current runtime checkout and its four changed camera assets are
preserved.

- Focused regression suite: 44 passed after the review fixes.
- Native lift driver: compiled in the candidate build directory, not executed.
- Five pure C++ checks passed: body/ramp limits, chassis motion, ordered chassis
  output, arm limits, arm trajectory. A separate body check also passed with
  -Wall -Wextra -Werror -UNDEBUG.
- Initial complete Agent + web Python run: 730 passed, 25 failed.
- Every one of those 25 failures reproduced in a separate unchanged ff14e6b
  checkout with the same read-only CAD/browser assets: 25 failed in 8.85 s.
- Final complete run after fixing two review findings: 733 passed, 25 failed
  in 145.88 s; the failed-test set exactly matches the unchanged-base failures.

## Reproduced baseline failures

These are existing failures, not a green whole-suite result. They are recorded
rather than mixed into this lift change. The list names every failed test:

- `agent/tests/test_battery_execution.py::BatteryExecutionTests::test_fixed_observer_uses_actual_wrist_path_length`
- `agent/tests/test_chassis_retreat.py::RetreatTests::test_real_controller_keeps_task_owner_and_arm_lift_while_using_native_transport`
- `agent/tests/test_coarse_alignment.py::CoarseTests::test_fixed_lift_plan_does_not_require_pre_lift_observer_or_tcp`
- `web/tests/test_battery_server_wiring.py::BatteryServerWiringTests::test_site_start_enables_valid_policy_and_arm_trajectories`
- `web/tests/test_battery_step_preparation.py::PreparationTests::test_coarse_step_is_assembled_from_frozen_both_arm_feedback`
- `web/tests/test_battery_step_preparation.py::PreparationTests::test_guided_observer_revalidates_actual_lift_using_cad_clearance`
- `web/tests/test_competition_http.py::CompetitionNavigationHTTPTests::test_loopback_navigation_rotates_ownership_and_manual_remains_blocked`
- `web/tests/test_competition_ownership.py::CompetitionOwnershipTests::test_external_stop_interrupts_whole_competition_and_cannot_rearm_navigation`
- `web/tests/test_competition_ownership.py::CompetitionOwnershipTests::test_manual_blocked_between_navigation_and_grasp_without_motion_reservation`
- `web/tests/test_competition_ownership.py::CompetitionOwnershipTests::test_standalone_navigation_unchanged_and_release_requires_clean_actuators`
- `web/tests/test_control.py::ControlTests::test_chassis_timeout_sends_zero_preserving_body`
- `web/tests/test_control.py::ControlTests::test_explicit_brake_still_publishes_when_no_web_motion_is_active`
- `web/tests/test_control.py::ControlTests::test_observation_mode_blocks_every_actuator_without_writes`
- `web/tests/test_control.py::ControlTests::test_page_sequences_survive_refresh_and_do_not_share_counters`
- `web/tests/test_control.py::ControlTests::test_requested_chassis_limits_allow_positive_values_and_reject_overflow`
- `web/tests/test_control.py::ControlTests::test_stale_brake_stops_without_rewinding_sequence`
- `web/tests/test_control.py::ControlTests::test_stop_wins_over_late_motion_and_other_clients`
- `web/tests/test_handeye_steps.py::WorkflowTests::test_only_owner_click_can_consume_one_ticket`
- `web/tests/test_handeye_steps.py::WorkflowTests::test_probe_failure_releases_depth_without_motion`
- `web/tests/test_http.py::HTTPTests::test_chassis_vector_reaches_adapter_once_and_keeps_limits_and_brake`
- `web/tests/test_http.py::HTTPTests::test_page_scoped_wasd_and_stale_brake_through_http`
- `web/tests/test_motion_owner.py::MotionOwnerTests::test_active_chassis_cannot_be_taken_over`
- `web/tests/test_motion_owner.py::MotionOwnerTests::test_all_other_motion_is_blocked_but_owner_uses_existing_validation`
- `web/tests/test_motion_owner.py::MotionOwnerTests::test_other_browser_stop_preempts_owned_chassis`
- `web/tests/test_motion_owner.py::MotionOwnerTests::test_stops_interrupt_old_task_and_cannot_be_undone_by_owner`

## Regression evidence

Tests first reproduced fractional acceptance truncation, equal-target overshoot,
abbreviated settling and legacy policy normalization. The review added failing
cases for one cached body sample being counted as multiple samples when arm
stamps advance, post-stop height drift, and a gripper-submission race. These now
pass in the focused run. Deliberately lower standalone acceptance and later
observer-motion drift keep their existing contracts.

## Physical validation still required

Measure stopping and holding through the existing controller under the actual
load, in a bounded movement approved by an onsite observer. The position P gain,
gravity feedforward, torque limit and default reference rate remain unchanged;
no reduction in physical endpoint error or faster motion is claimed.


## Supervised baseline and prepared endpoint helper

The later approved physical-controller baseline is recorded separately in
[lift-low-baseline-20261119.md](lift-low-baseline-20261119.md). That trial used
the existing web controller and did not change gain, speed or driver ownership.
It completed its command cycle but failed measured endpoint accuracy.

The new pure endpoint helper is not connected to hardware. Its ten tests and
the previous 44 focused regressions pass: 54 total. The final complete suite
has 743 passes and the same 25 baseline failures (no new failed test names).
Fresh review reproduced a short stable-window bug and mean-only arrival bug;
both have regression tests and were fixed before any compensation trial.
