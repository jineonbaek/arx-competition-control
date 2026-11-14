# Existing navigation integration implementation plan

**Goal:** Connect the colleague's existing route and handoff to map.html, preserving standalone navigation and every grasp action/tolerance.

**Architecture:** Reuse the domain-192 map observer to publish existing route start/clear topics and read route progress. A server-side adapter uses the existing TaskClient/HandoffHub protocol. Separate whole-competition manual exclusion from short-lived actuator ownership so navigation can verify motion_reserved=false between jobs.

**Authority:** User approved implementation, tests, commit, deployment and web reload; no physical task starts. Work inline in an isolated worktree. No changes under navigation/, agent/ or execution configuration. Preserve concurrent colleagues' changes.

## Tasks

- [x] Test and implement Controller competition ownership: manual exclusion, local navigation channel, stage handover, emergency stop and cleanup. Preserve standalone command semantics.
- [x] Test and implement observer route relay + navigation session: inert construction/GET, configured route only, lease, exact visit identities, stable event retries, real arrival, cancellation and no replay.
- [x] Connect modules to CompetitionTask and server; reuse existing carry_ready evidence when available, never execute another transport motion. Show navigation progress on map.html.
- [x] Run isolated HTTP/protocol and lifecycle tests, existing control/grasp regressions and JS checks. No tests against production movement endpoints.
- [ ] Review diff and narrow publication whitelist. Commit only integration changes. Apply compatible hunks to current main, verify protected diffs/hashes, reload only idle web/our observer, verify GETs.

## Review focus

Manual commands must remain rejected during token-free navigation handover; old clients/tokens and late replies cannot resume a stopped competition. No false complete on missing carry/placement or unknown receipts. Failures must cancel the actual route without touching its definitions/algorithm. Observer reload must not replay pending start commands. Concurrent server.py work must remain byte-for-byte outside our small wiring hunks.

## Evidence

Checkpoint: checkpoint/map-navigation-before-20261115-023924 (f6209e5). Untracked archive and manifest under /home/arx/arx-nav-integration-checkpoint-20261115-023924. Worktree /tmp/arx-map-navigation-20261115 at 0d25b60. Baseline: 52 control/competition/trajectory tests passed.

Ruling: reuse the latest existing grasp carry_ready receipt as the carry module; 0d25b60 already performs transport return after extraction. Do not modify or repeat it.

Verified: 170 Python tests including the committed server wiring passed; 12 JS tests passed; intercepted desktop/mobile browser check passed with no JS errors. No production motion requests sent. Deployment confirmation will be recorded in the external checkpoint directory.
