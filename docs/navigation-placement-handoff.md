# Navigation to placement handoff

The 9983 competition controller consumes the navigation team's existing local HTTP protocol at `http://127.0.0.1:9985`. The navigation algorithm, maps and ROS route executor remain owned by that team.

## Counterpart contract

| Endpoint | Purpose |
| --- | --- |
| `POST /v1/worker` | Register battery worker for `point_1` and `point_2`; receive a process-local bearer token and 2 second lease |
| `POST /v1/heartbeat` | Renew the same worker lease every 0.2 seconds |
| `GET /v1/state` | Read route/task status without registering or changing navigation |
| `GET /v1/task` | Read the current notified visit using the worker token |
| `POST /v1/task` | Send idempotent accept, fail, cancelled or complete events |

Every event binds `mission_id`, `visit_id`, `point_id` and a stable `request_id`. Retries reuse the same request ID. A reply lost after acceptance must never replay the arm action.

The route must contain task points `point_1` then `point_2`, and expose its existing route definition/status through the ROS observer. The worker accepts only the current map, route-point and mission identity after the navigation end has notified a stopped arrival.

## Real place adapter

The competition `place` stage now uses `NavigationPlacement` rather than the T2 recorded preview. It:

1. Requires a real, matching `point_2` visit and the competition's current motion token.
2. Reads the operator-selected cabinet reference from `web/hardware/runtime/placement-references/active.json`.
3. Observes D435 cabinet position and orientation and applies bounded, measured chassis corrections using the existing task-scoped chassis transport.
4. Confirms stopped chassis position and records `reports/visual-chassis-active/handoff.json` with the navigation visit ID.
5. Starts the existing right-arm 160 mm placement with the same motion token. It does not request or release a second owner.
6. Requires the complete insertion, release and return receipt before reporting module completion.
7. Lets the competition controller release motion ownership before the existing navigation session sends `complete(control_released=true, navigation_ready=true)`.
8. Waits for the navigation end's acknowledgement. The navigation end alone validates its sensor/stop proofs and advances the remaining route.

Placement uses its separate zero-radian lift reference; grasp A/B/C/PlanD settings are unchanged. The communication adapter does not reposition the lift: before any chassis correction or arm dispatch, fresh lift feedback must be within 0.10 rad of the placement reference. If carrying preserves a different grasp height, an independently validated placement-height preparation is still required before accepting the placement stage. The adapter reports this missing prerequisite rather than silently attempting insertion at grasp height.

A missing or invalid cabinet reference, stale visual handoff, invalid ownership, degraded arm result or changed visit reports failure, never a successful completion. Both request IDs and consumed `(mission_id, visit_id)` pairs are retained, so a changed request ID cannot push or release twice on the same visit.

## Read-only integration checks

Open `/map.html` to establish a web session, then read `/api/competition-task`. A live installation reports `placement_backend: visual_chassis_d405_insert_160`, `placement_readiness`, and the navigation session state. Merely opening or polling these endpoints does not register a worker, start a route, or command a motor.

`placement_readiness.reference_valid=false` means the cabinet reference must be recorded at the accepted cabinet-front position using `/visual-docking.html`. Do not turn an arbitrary current image into a reference to satisfy readiness.

A refused 9985 connection means the navigation team's task runtime is not listening. A successful HTTP connection alone does not establish valid map localization, a prepared route, a stopped arrival, or physical placement success.

## Verification

- `test_navigation_placement`: admission, retained ownership, full completion, duplicate suppression, invalid reference, wrong point and degraded result.
- `test_navigation_placement_protocol`: real local HTTP accept/fail/complete and acknowledgement contract on an ephemeral test port, without hardware.
- `test_navigation_owned_placement`: borrowed token, stale admission rejection and ownership retention on early driver failure.

These tests do not represent a physical navigation or insertion trial.

### Deployment verification, 2026-11-20

The 9983 API reports `placement_backend=visual_chassis_d405_insert_160`; the `place` stage is connected and no longer labeled as a recorded preview. All four grasp modes remain present. Focused Python tests: 36/36. Focused map/camera/placement JavaScript tests: 14/14. No real navigation route or placement was executed.

The navigation team's 9985 listener was still offline at verification. The saved cabinet reference `ref-1795094043742-95e73fde` has `d435.valid=false`; readiness reports this rather than substituting a new unconfirmed reference. Placement-height preparation must satisfy the separate zero-radian requirement before dispatch.

The host's pre-existing `test_navigation_session` suite is 5/8 on both the unmodified baseline and the integration check. Its existing arm-preset preparation now runs before navigation, while these older fixtures do not provide valid arm feedback. The unchanged failing cases are:

- `test_missing_place_cancels_route_and_returns_manual_without_fake_complete`
- `test_stop_during_grasp_cancels_navigation_too`
- `test_total_sequence_real_handoff_never_opens_manual_between_jobs`

The current host's `competition_task.py` and its arm-preset behavior were deliberately preserved. The new real-HTTP placement protocol tests exercise the new adapter independently of those pre-existing fixture failures.
