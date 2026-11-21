# Plan F grasp recovery (local implementation only)

User confirmed preobservation 2 is the observation posture at the start of this
run, after lift/dock preparation. It is never the placement D405 point.

Plan F ignores strict target-position/orientation error and lift-offset gates;
these become recorded diagnostics. It still requires a completed driver reference,
fresh feedback, measured stationary dwell, valid ownership and clearance. Unknown
execution, cancellation, stale feedback, moving chassis, uncertain grip and
unconfirmed contact stop never initiate automatic recovery.

Before closure, recoverable perception/planning failures return along the measured
completed approach route to point 2, then select a different observation posture
(J4 +4, -4, J5 +4, -4 degrees relative to point 2). Each route is checked against
joint limits and the CAD column at current lift height. A new D405 observation is
mandatory; cached A/B targets are not used. Four candidate attempts maximum;
unclear/nonrecoverable failures stop. No repeat after possible closure.

Point 2 is immutable within a run. Recovery command identity/status is persisted
before effects; uncertain or restarted attempts never replay. Motion state and
actual error are evidence, never fabricated success. Other grasp modes and
placement contact-stop handling retain existing behavior.

No deployment, remote Git push or physical motion test is authorized in this task.

## Completed review decisions

Recovery motions are different from normal pick arrival: returning to point 2
and selecting a candidate require measured joint endpoints (0.5 degrees) and
stationarity. Reference completion alone never proves a changed viewpoint.
Two completed approaches/corrections without measured joint progress above 0.1
degrees select posture recovery; productive legacy corrections are unchanged.

Every recovery observation, preview and plan enforces the post-recovery camera
time barrier. Approach traces are deduplicated by execution ID. Recovery IDs are
checked against all durable attempts; missing origins in old mid-run state are
rejected. The measured start connector is checked before gripper opening and
trajectory dispatch. A changing pose with unchanged paired feedback time cannot
permit closure. Recovery failures use the existing shared unresolved-stop latch
so neither standalone nor competition cleanup can release ownership prematurely.

Local verification: 33 new regressions and 242 selected tests passed. Eleven
pre-existing test failures were reproduced on the untouched local snapshot and
excluded from the green selected run; no full-suite success is claimed. No
hardware movement, deployment, driver restart or remote Git push performed.
