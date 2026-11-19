# Lift accuracy and speed implementation plan

> **For agentic workers:** Use superpowers:executing-plans for inline execution.

**Goal:** Fix premature web lift completion and prepare bounded speed selection
without changing the live controller.

**Architecture:** Preserve exact browser targets through policy construction;
use existing strict feedback monitoring at the endpoint. Validate a startup-only
reference rate before vendor SDK construction and retain the current default.

**Tech Stack:** Python unittest/pytest, C++17, ROS 2 Jazzy.

**Spec:** `docs/lift-accuracy-speed.md`.

## Global constraints

- Independent branch/worktree; no live service, CAN, firmware or gain changes.
- Exact target, current 0.02 rad arrival tolerance and configured settle window.
- Ramp default 0.5, allowed `(0, 1.0]` SDK rad/s, homing 0.5 rad/s.
- No push or claim of physical improvement without validation.

## Review focus

- Old saved settings must not reintroduce rounded acceptance (Task 1 test).
- No-settings startup must also use exact acceptance (Task 1 test).
- Equality must not accept overshoot or abbreviate settling (Task 1 tests).
- Explicit lower coarse thresholds must stay compatible (existing Task 1 test).
- Invalid speed must not mutate config; runtime changes forbidden (Task 2 tests
  and read-only ROS descriptor/native build review).

### Task 1: Exact web height and arrival

Files: `web/src/battery_task.py`, `agent/battery_production.py`,
`agent/coarse_execution.py`, `config/battery-execution.json`, corresponding
`web/tests/test_battery_task.py`, `web/tests/test_coarse_step.py`,
`agent/tests/test_battery_production.py`.

Interface: task settings and coarse policy retain existing fields; equal
`lift_acceptance_rad` selects strict `ArrivalMonitor` behavior.

- [x] Write/run failing tests: 6.6 remains 6.6; legacy settings and policy accept
  only the selected height; equal-target overshoot cannot complete; equality
  retains the 0.2 s fixture settling window.
- [x] Preserve exact web acceptance and normalize only the web workflow;
  keep standalone lower-threshold semantics.
- [x] Run focused tests and Python suites; document results and commit.

### Task 2: Bounded startup ramp

Files: `web/hardware/slow_lift_config.hpp`, `lift_controller_web.cpp`,
`test_body_limits.cpp`, `web/README.md`, feature documentation.

Interface: `apply_slow_lift_config(Config&, double rate=0.5)` validates then
sets `lift_max_vel=rate/400`, keeping homing at 0.5. ROS exposes read-only
`lift_target_ramp_rad_s`.

- [x] Add/run failing C++ tests for 400-tick default/1.0 ramp distance, invalid
  zero/negative/over-limit/nonfinite rates and no partial mutation.
- [x] Implement helper validation and the startup-only parameter before SDK
  construction; use the selected rate in startup logging.
- [x] Run pure C++ checks and build the native driver separately, without
  executing it; update documentation and commit after review.

## Progress

- Active checkout preserved. Baseline initially lacked ignored CAD resources;
  attach read-only site assets and rerun before interpreting test failures.


- Focused baseline: 36 passed once site assets were linked read-only.
- RED: six endpoint/policy assertions plus missing configurable-ramp API; review
  exposed three additional failing handoff/timestamp scenarios.
- GREEN: 44 focused Python tests, five pure C++ checks; native driver built only.
- Final full suite: 733 passed, 25 failed; identical failed-test set on base.
- Review findings fixed: body sample timestamps and precise post-stop handoff.
- Both implementation tasks form one lift candidate delivery commit. Source
  commit is allowed; no push, deployment, homing or physical test was performed.
- Physical servo tuning and higher-rate validation remain outstanding.
