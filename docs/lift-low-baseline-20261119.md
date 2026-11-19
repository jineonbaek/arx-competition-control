# Low-position lift baseline and bounded correction proposal

## Completed physical-controller test

On 2026-11-19, after onsite approval, the existing web controller performed
0.45 → 0.95 → 0.45 SDK rad. The originally proposed high-position test was not
executed because a fresh check found the platform had already been lowered.

The original body driver PID 5922 remained running. Speed, gains, torque limit,
motor mode and calibration were unchanged. Commands used the existing
`/api/command` web route; read-only ROS parameter queries checked the accepted
reference. There was no second SDK owner, direct CAN command or restart.

| Stage | SDK reference | Last 2 s mean feedback | Error |
|---|---:|---:|---:|
| Initial hold | 0.450000 | 0.481874 | +0.031874 |
| Raised, held 8 s | 0.950000 | 0.704013 | -0.245987 |
| Returned, held 8 s | 0.450000 | 0.702116 | +0.252116 |

The raised reference increased by 0.5, while measured height increased only
0.222139. On return, feedback decreased only 0.001897. The final measured
height remained 0.220242 above its initial value. The cycle completed, but
endpoint accuracy failed the existing 0.02 rad tolerance. Fresh samples were
received throughout; command epochs were 2386, 2387 and 2388, matching the two
test commands without intervening controller commands.

The onsite operator reported no unusual sound, jamming or abnormal heating.
This does not prove a specific mechanical cause. The result shows a persistent,
direction-dependent dead-zone behavior consistent with the earlier logs; it is
not a millimetre-height calibration. A single fixed offset cannot correct both
directions in this test.

Raw feedback, command receipts and analysis remain in the ignored local folder
`.superpowers/lift-accuracy-speed/low-baseline-20261119-114609/` in the candidate
worktree. Runtime recordings are deliberately excluded from Git.

## Prepared experimental helper

`agent/lift_endpoint.py` contains two pure calculations and cannot connect to or
command hardware. It is not wired to the live webpage or driver:

- `settled_height(samples, target=...)` requires distinct, fresh, continuous body
  packets covering at least 1.5 seconds, with total height span at most 0.005 rad.
  It includes the packet bracketing the beginning of the window and returns the
  measurement furthest from the desired target, so an average cannot conceal
  an endpoint outside tolerance.
- `next_reference(target, reference, measured, corrections=...)` computes a
  reference proposal from the measured residual. Single-step and total deviation
  from target are each bounded to 0.30 rad, with at most three corrections.
  Reference limits or exhausted attempts return an explicit failure rather than
  successful arrival. Only measured error within 0.02 rad establishes arrival.

The supervising caller must collect a new stable window after each accepted
command, verify the actual reference and control ownership, enforce a finite
wait, and request the existing lift-stop service through the webpage on faults.
There is no database, API or dependency change. Importing/testing the helper is
inert and requires no robot, SDK, ROS node or cloud service.

## Next supervised trial — not yet authorized or executed

Proposed desired feedback heights: **0.45 → 0.95 → 0.45 SDK rad**. Keep the
existing speed and gains. Permit SDK references only within **0.15–1.25 rad**,
with the 0.30 rad step/offset and three-correction limits above.

The current data suggest a first downward reference near 0.198 and an upward
reference near 1.196, but those are predictions, not proven endpoints. Every
proposal must be recomputed from new settled feedback. After each successful
correction, restore the nominal reference and verify that the actual height
stays within tolerance; do not silently leave a hidden permanent offset.

Abort on stale/lost/regressing feedback, changed control ownership or target,
unsettled samples, exhausted bounds/counts, or an uncertain command reply.
Do not start this trial without approval of the wider reference interval and
continued onsite emergency-stop supervision. Verify the current starting state
again before issuing any command. No increase to speed is part of this trial.

## Validation and remaining work

54 focused software tests pass, including the ten new helper regressions and
the previous 44. Review found and tests reproduced two settling-boundary issues;
both were fixed before any compensation trial. The complete Agent/web suite has 743 passes and the same 25 unchanged-base
failures, with no new failed test names.

Physical compensated accuracy, holding after restoring nominal reference,
manual/automatic runtime integration, and increased speed remain unproven.
Keep the candidate separate from the running checkout until those steps are
explicitly reviewed and validated.
