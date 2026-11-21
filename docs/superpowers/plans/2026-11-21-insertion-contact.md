# Insertion Contact Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Continue bounded insertion until a contact or endpoint; permit a
visually confirmed seated contact to finish automatically without releasing on
an early jam.

**Architecture:** A pure contact monitor consumes position/current feedback.
The executor confirms an intentional contact stop before handing it to the
existing visual seating state machine. Ordinary faults remain faults.

**Tech Stack:** Python, NumPy, unittest, existing ROS 2/SDK position execution.

**Spec:** docs/superpowers/specs/2026-11-21-insertion-contact-design.md

## Global Constraints

- Preserve existing chassis/platform state and motor drivers.
- No direct force measurement or calibrated force limit is claimed.
- Only a confirmed contact plus fresh D405 seating evidence authorizes release.
- Do not send physical motion to validate this software change.

## Review Focus

- A slow nominal quintic start/finish must not look like a contact.
- Duplicate/stale feedback cannot accumulate a valid stop window.
- A failed stop, driver replacement or user cancellation cannot release.
- Partial travel must not retain unexecuted forward waypoints in its return path.
- Current/body depth alone cannot release a battery jammed before the seat zone.

### Task 1: Feedback and pure contact monitor

Files: web/src/control.py, ros_adapter.py, insertion_contact.py;
web/tests/test_insertion_contact.py, test_control.py.

- [x] Write failing tests for current propagation/clearing and feedback sequences.
- [x] Observe RED; implement optional current propagation and ContactMonitor.
- [x] Verify progress, loaded baseline, transient current spikes, missing current,
  sustained contact, fallback stall and stale/duplicate samples.

Interfaces: ContactMonitor.sample(state, driver, now) produces a
typed InsertionContact carrying diagnostic evidence; it submits no commands.

### Task 2: Stop confirmation and insertion integration

Files: web/src/docking_motion.py, fast_visual_level.py, current_axis_push.py,
mouth_push.py; focused web/tests tests.

- [x] Write failing tests for confirmed contact stop, failed stop, early contact,
  near-seat visual release, invalid visual evidence and a reached-only return path.
- [x] Observe RED; integrate contact monitoring in the execution callback outside
  controller preflight, acknowledge/settle the stop, and preserve the actual path.
- [x] Hand confirmed contact to run_fixed_stroke; no replay after contact. Only
  near-seat confirmed contact can relax the nominal travel check to its seat zone.
- [x] Verify normal fixed-stroke and cancellation regressions.

### Task 3: Review, validation and deployment

- [x] Run focused tests, relevant full web/agent test suites and compile checks.
- [x] Request a fresh review of the complete change; fix substantive findings.
- [x] Preserve a local patch and receipt; compare remote hashes before deployment.
- [ ] Deploy only changed files, reload an idle web service, and read back current
  telemetry/health. Do not actuate the robot or claim physical validation.
