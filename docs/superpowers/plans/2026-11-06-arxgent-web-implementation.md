# arxgent Web Implementation Plan

> Execute inline with superpowers:executing-plans; one final independent code review. The user supplied the screenshot, name, placement and real-dialogue acceptance test and requested implementation directly.

**Goal:** Add arxgent chat above chassis controls on the existing 9983 page and validate real Qwen vision dialogue from that page.

**Architecture:** Existing ThreadingHTTPServer hosts same-origin async chat endpoints. A bounded chat manager reuses AgentRuntime and immutable observations under an enforced read-only policy. Existing browser cookie owns a conversation; frontend polls without blocking control feedback.

**Tech Stack:** Python standard library, existing vanilla JS/CSS, installed Playwright for browser verification.

**Spec:** docs/superpowers/specs/2026-11-06-arxgent-web-design.md

## Constraints / scope

- Continue branch codex/qwen-agent-framework in /home/arx/arx-agent-framework; base957e529. Explicit local commits per verified feature; no push/merge.
- Whitelist source changes: agent/runtime.py; agent/observations.py; agent/tools.py; web/src/arxgent.py; web/src/server.py; web/public/index.html; web/public/js/arxgent.js; web/public/css/arxgent.css; corresponding new tests/docs.
- Chat only get_system_status / observe_scene / read_observation, motion disabled, no robot backend.
- Preserve existing unrelated dirty files in active /home/arx/arx-competition-control. Do not change launcher, control, ROS adapter, hardware or camera code.
- User explicitly requested live-page change/verification, permitting selective publication and web-only reload after review. This overrides earlier generic no-deployment default for this scoped page update only.
- API keys remain server-side in ignored mode600 configuration; no key in HTTP, JS, tests, logs or commits.

## Review focus

1. Browser conversation isolation, same-origin authorization and response redaction.
2. Duplicate submission / polling / cancellation / page refresh must not cause repeated API jobs or stuck busy state.
3. Read-only tool policy enforced on execution and retained on future turns, not merely hidden from schema.
4. Chat keyboard input must not trigger chassis motion; changes must not alter manual control behavior.
5. Reload must target only the verified web PID and preserve other processes and dirty source hashes.

## Task 1: Chat manager and HTTP endpoints

- [x] Write failing tests for asynchronous send, owned histories, multi-turn context, validation, request deduplication, busy limits, cancellation, readonly policy and HTTP guards (5 initial missing-module failures).
- [x] Add minimal AgentRuntime continuation method with completed-turn guard and per-turn budget reset; preserve all prior call receipts.
- [x] Implement manager and optional handler injection; default existing handler tests remain usable without model credentials.
- [x] Run new tests plus Agent/web tests, check scoped diff and commit (31 Agent +85 web Python passed on ROG).

## Task 2: UI and live-page validation

- [x] Add panel before CHASSIS with existing visual tokens, accessible input/transcript/status, send/cancel/new-chat and suggested observation prompt.
- [x] Add frontend tests for safe text rendering / keyboard / pending and API behavior; check desktop and mobile layout in browser (3 Node tests; real Chromium offline fixture verifies 2 turns, refresh, keyboard guard, literal markup, cancel and390px viewport).
- [x] Run all related tests, review source diff and commit (31 Agent,85 web Python,15 Node; offline browser no page errors or motion requests).
- [x] Final independent review; fix material findings with regression tests and scoped commit (3 Important findings;4 regression tests observed failing then passing).
- [x] Compare clean target files against Git base and preserve dirty file hashes; selectively publish committed files and reload only the web process (20 source files frozen at6cf3529; original6dirty-file hashes unchanged; web3744->277817; camera/control PIDs3643,3655,4391,4457,4526 preserved).
- [x] Verify browser real Qwen tool/image dialogue and follow-up; record commits, tests, runtime evidence and remaining limits (22.65s state+3camera reply,10.44s context follow-up,4messages retained on browser reload;0 control requests and0 browser page errors).

## Execution record

- Current active server uses /home/arx/arx-competition-control/web/src/server.py, main e744156. Existing Agent branch957e529. Launcher does not continuously respawn web; RosAdapter.close may issue a chassis stop, so deployment requires chassis inactive and preserves existing control processes.
- Browser automation: Python Playwright and bundled Chromium are already installed locally. No new runtime dependency needed.
- Backend commit b205f33 passed31 Agent+85 web tests. All writes remain on the independent development branch so far.
- User specified exact placement/name and requested implementation/real-page verification; routine visual and architecture details were resolved using existing UI/service patterns without another approval round.
- Final review: live status now reads bounded sanitized /api/state feedback, source timestamps and freshness, with explicit unavailable state; no guessed height units. The original mission/config status remains distinct.
- Final review: continuation uses persistent per-turn internal call namespaces while retaining original provider IDs on the API protocol, so new-frame requests cannot reuse a prior turn's observation cache. Same-turn request deduplication is retained.
- Final review: expired idle sessions are reclaimed after1hour; when the32-session bound is reached, unused spectator sessions are evicted before refusing a new client. Ongoing jobs and recent conversation history are preserved.
- Functional commits: b205f33 backend,ea308ae UI,6cf3529 review fixes. Final checks:31 Agent+89 web Python+15 Node, offline Chromium workflow passed; live browser confirmed arxgent identity and previous three camera names.
- Deployment was explicitly authorized by the user's request to modify and verify the current9983 page. Source overlay is installed in active main e744156 without merging or committing unrelated work. The development branch remains the source-of-truth for these changes; main has the intentional runtime overlay plus its original dirty work.
- Limits: chat remains read-only; browser refresh restores conversations within the running web process, but web-process restart starts a new browser conversation while private run evidence remains on disk. No physical manipulation validation performed.
