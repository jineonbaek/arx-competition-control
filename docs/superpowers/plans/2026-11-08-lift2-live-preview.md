# LIFT2 read-only live preview implementation plan

Goal: Add the official LIFT2 assembly above the existing 9983 controls, following real left/right joint and lift feedback. User explicitly selected the TH URDF/STL rendering approach; no TH controls or command transport.
Architecture: Locally served pinned Three.js, URDFLoader and official meshes. An independent GET-only module polls /api/state, hides stale components, and maps radians directly to arm joints. Lift displacement uses the existing nominal 41.54 rad/m solely for uncalibrated display. Keep original official URDF and disable display joint clamping, never change hardware limits.
Scope: scripts/prepare_lift2_preview.py, web/models-lift2.json, web/public/index.html, web/public/css/robot-preview.css, web/public/js/robot-preview.js, web/public/js/robot-preview-state.js, web/tests/test_lift2_assets.py, web/tests/robot-preview.test.mjs, web/tests/check_robot_preview_browser.py, web/LIFT2-PREVIEW.md, .gitignore, this plan.
Assets under web/public/models/lift2 and web/public/vendor/lift2-preview are generated and ignored; no vendor SDK, robot drivers, backend, camera config or other dirty files are edited.

## Task 1: prepare reproducible local assets
- [x] Integrity/URDF tests: expected missing-script failures; portable path regression RED then GREEN; all 3 tests pass.
- [x] Added pinned downloader, manifest and instructions.
- [x] Installed 21 meshes + URDF + 4 libraries; all 26 installed file hashes and URDF references checked; STL extents measured.
- [x] Task 1 diff reviewed; source/preparation commit 08c60c7.

## Task 2: add live read-only viewer
- [x] Six mapping/isolation tests first failed for missing implementation, then passed.
- [x] Added top panel and independent renderer/GET-only feedback; no backend or driver changes.
- [x] Browser verified actual WebGL + 21 meshes, live reads, fixture mappings, frozen whole responses, stale feedback, narrow viewport, GET-only requests.
- [x] Final suites: Node 30/30, Python 157/157; browser six checks pass; HTTP resources 200; all four server/driver PIDs and start ticks unchanged.
- [x] Fresh read-only review completed; both material findings fixed RED-to-GREEN; scoped local commit, no push.

Review focus: no zero pose presented as live before valid feedback; independent freshness per component; feedback never clamped to control targets; invalid lift does not invent a height; asset failure cannot break existing controls; model is not collision protection.

Final review: fresh read-only reviewer found (1) manifest self-hash on repeat preparation and (2) full stale-response replay resets clock. Both regression tests failed first, then passed after fixes. Repeated-preparation fix committed bfa9715. Lift conversion and CAD joint mapping remain explicitly uncalibrated display assumptions; no hardware motion attempted.
