# Empty 30mm cabinet rehearsal
User requests a physical empty-gripper rehearsal, outside alignment then fixed-world-Z insertion. No battery release, no lift/left-arm movement, no push, original projects protected.

1. Preserve a frozen scene, TCP hash, current body/left/right/gripper state, nominal closed-jaw full mesh continuous-sweep certificate (>10mm), and explicit source age. Latest live scene must be recaptured before authorizing a run.
2. Add a one-shot execution wrapper using the existing Controller motion reservation and driver segment_quintic_scalar transport. Split long outside alignment into stages below driver 60s duration. Insert at most 30mm then reverse to the outside point. Bundle and separate local approval are hash-bound; no coordinate input accepted over HTTP. No approval means no motion.
3. Tests first: altered/expired bundles, discontinuity, missing certificate, wrong ID, stale/moved initial state, replay, successful measured arrival, mid-run body drift and cleanup. Require current reference tracking and full live source hashes before physical execution.
4. Add same-origin/session guarded start/status/cancel API; close cancels active work. Existing preview and floor placement gates remain disabled. Deploy only after focused and full tests; no driver restart.
5. Run only after current scene/certificate plus现场 empty-gripper/stop-monitor confirmation. Any unmet gate is reported; no bypass or asynchronous target stepping.
