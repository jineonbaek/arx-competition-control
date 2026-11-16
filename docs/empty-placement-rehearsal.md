# Empty-gripper cabinet rehearsal

Scope: no battery carried or released. Outside alignment may change height; insertion tool reference is horizontal with world Z fixed. Maximum TCP shallow depth30mm, then reverse the same checked joint curve to the outside point. Original LIFT/ROS2 projects untouched, no push.

## Verified evidence (2026-11-14)
- Actual right driver publishes enabled=true, ready=true for its segment_quintic_scalar trajectory interface. Reuses original Controller exclusive motion ownership, start/poll and stop. No point-by-point asynchronous substitute.
- Archived scene `eeaabb06a29c415ba7c904973294494c`: current lift10.989738rad has no accepted bounded prealignment IK. Conditional lift10rad has a path: first aligned joints approximately37.67/109.33/38.07/71.26/43.24/-89.67deg.
- Nominal closed-finger CAD shallow30mm path, both arms, modeled cabinet and depth-fitted tabletop continuous sweep: clearance lower10.029mm, sampled closest11.256mm. Not a physical accuracy measurement; unobserved obstacles and jaw calibration remain field conditions.
- Outside lowering from10.989738rad toward10rad was also checked down to9.5rad for tracking allowance, including both arms and moving body4. Nominal minimum lower133.557mm including fixed-base clearance.
- Conditional timing: two outside alignment segments34.24s each; shallow insertion31.90s; reverse31.90s; total132.28s. Joint reference limits3deg/s,6deg/s^2. Insertion reference TCP10mm/s; alignment20mm/s. Each driver message<60s.

## Execution gates
No motion on construction, GET, or absent local approval. `/api/cabinet-rehearsal` shares existing cookie + same-origin guards, accepts only a fixed run_id or cancellation. A local approval hashes the same exact bytes that are parsed as the bundle. A consumed run cannot replay. Live TCP, frozen scene, calibration, control configuration, cabinet and robot geometry are bound to hashes; recheck before each segment and periodically during execution.

Initial rest and post-completion/cancellation rest require new increasing feedback timestamps from both arms and body over a stable window. Cached `fresh=true` is insufficient. Guard rejects body/left-arm/gripper drift and right-arm reference tracking error (>1deg, >1mm modeled TCP or >0.3deg tool rotation). Source changes, manual stop, cancellation or lost feedback stop the run. Ownership is not released after unverified stopping. Completion reports empty rehearsal, never battery placed.

## Current deployment state
Default disabled. No approval file installed. After field confirmation, bounded low-speed lift adjustments brought feedback from10.99 to10.07rad followed by hold requests; the right arm remained stationary during preparation. Fresh scene and trajectory checks are required after any manual movement and before run authorization. Conditional lift10rad plan is not an approved live plan. Floor placement remains disabled.

Relevant implementation: web/src/cabinet_rehearsal.py; tests test_cabinet_rehearsal.py and test_cabinet_rehearsal_http.py. Reviewer findings on cached packets, file metadata and parse/hash races were reproduced and corrected. Startup import order has a clean-interpreter regression test.

## Authorized physical attempt

Run empty-live-e7b0c4c5 stopped during outside_1, before any insertion. Duration to stopped receipt4.50s; maximum observed right joint excursion0.328deg. Reference-start bracket from external samples implies tool orientation tracking error0.316–0.345deg near3.87s, exceeding the0.3deg rehearsal guard; modeled TCP position error0.842–0.915mm at that sample. This estimate is not an exact driver reference timestamp. All feedback remained fresh, frozen source hashes unchanged, lift near10.055rad. Recovery_required=false, motion reservation returned. Approval was consumed and archived; no automatic retry, no battery placed, no push. Field observation received: user confirmed stopped, no contact/noise/shaking.
