# Competition Agent Framework Implementation Plan

> **For agentic workers:** Execute inline using superpowers:executing-plans. Use TDD and preserve the existing robot workspace and processes.

**Goal:** Build the systematic task-agent framework described in the approved competition discussion, using Qwen3.8-Flash as the main LLM and real read-only camera observations as the first integration.

**Architecture:** Separate model transport, tool validation/execution, competition state, immutable observations, domain backends, and persistent run records. All competition stages have contracts and explicit availability; unimplemented capabilities cannot advance state or report success. The existing camera owner and robot control services retain ownership.

**Tech Stack:** Python 3.12+ standard library, Chat Completions with streamed/non-streamed responses, JSON schemas, JSON/JSONL, existing camera HTTP GET endpoints. No additional agent framework dependency.

**Spec:** `D:/Documents/meituan_comp_final/决赛机器人_Agent与视觉抓取方案_讨论总结_2026-11-06.md`, especially sections 5-19 and 23-27. User explicitly requested a systematic framework and authorized implementation before an API key is available.

## Global Constraints

- Cloud model: `qwen3.8-flash`; local profile remains `Qwen3.5-2B`, quantization/runtime not chosen.
- Credentials come from environment variables and are never written to logs/config examples.
- Existing camera capture is reused through HTTP GET; no camera pipeline, ROS, SDK or CAN owner is started.
- Physical backends default unavailable and motion disabled. No fabricated grasp/navigation/insertion success.
- Active checkout `/home/arx/arx-competition-control` remains on its current branch and preserves unrelated changes.
- Development worktree: `/home/arx/arx-agent-framework`, branch `codex/qwen-agent-framework` from `e744156`.
- Local review copy: `D:/Documents/meituan_comp_final/agent-framework`.
- No live model call until the user configures credentials. Offline fixtures are identified as test fixtures.

## Review Focus

1. Malformed/unknown tool calls must be rejected before a backend executes.
2. Received/finished motion must not imply a confirmed grasp or task completion.
3. A lost execution reply must retain a pending receipt, block dependent actions, and never replay blindly.
4. Camera filenames, timestamps, dimensions and image identity must be validated; current captures do not promise cross-camera or robot-state synchronization.
5. Partial model streams, missing credentials, cancellation and step budgets must fail explicitly without inventing a final result.

## File Map

- `agent/config.py`: cloud/local profile and bounded runtime settings.
- `agent/storage.py`: exclusive run ownership, events, state, observations, call receipts and cancellation.
- `agent/observations.py`: current console HTTP GET integration and immutable snapshots.
- `agent/backends.py`: explicit external domain-service contract, availability and receipts.
- `agent/mission.py`: competition stages and evidence requirements.
- `agent/tools.py`: schemas, strict validation, stage guards and capability dispatch.
- `agent/model.py`: Qwen transport, streamed tool-call assembly and errors.
- `agent/runtime.py`: model/tool loop, budgets, history, image attachments and run outcomes.
- `agent/__main__.py`: doctor, tools, observe, run, resume/status and cancel entry points.
- `agent/config.example.json`, `agent/.env.example`, `agent/README.md`: configuration, use, limits and next adapter contracts.
- `agent/tests/`: real local HTTP fixtures, protocol tests, state/evidence tests and end-to-end offline loop tests.

## Task 1: Task State, Storage, Tools and Domain Contracts

**Interfaces:** `RunStore`, `Mission`, `ToolRegistry`, `BackendRouter`; structured dictionaries with explicit error codes and execution status.

- [x] Write tests for stage prerequisites, unavailable capabilities, strict arguments, idempotency, pending execution and evidence-based completion.
- [x] Run the tests and record expected failure before implementation (11 missing-framework failures).
- [x] Implement the smallest complete persistent state/tool/backend layer.
- [x] Run the task tests until green (11 passed on Windows and ROG).

## Task 2: Real Observation Adapter and Model Transport

**Interfaces:** `ConsoleObservations.capture(cameras)`, `QwenClient.complete(messages, tools)`, normalized assistant message retaining tool calls and reasoning fields.

- [x] Write tests against real local HTTP servers for fresh/stale images, path traversal, source-image binding, missing depth/extrinsics, streamed tool-call chunks and partial-stream rejection.
- [x] Run the tests and record expected failure (5 missing-module errors).
- [x] Implement camera GET capture, profiles and model transport without new dependencies.
- [x] Run the task tests until green (16 total on ROG; live raw and annotated snapshots from all 3 cameras, 640x480).

## Task 3: System Runtime, CLI and Verification

**Interfaces:** `AgentRuntime.run(instruction)`, CLI commands, durable event and outcome records.

- [x] Write integration tests for multi-round model/tool exchanges, vision attachments, budgets, cancellation, unknown execution/resume and truthful final outcomes (6 new tests observed failing before implementation).
- [x] Implement orchestration, CLI, configuration examples and operational documentation.
- [x] Run all agent tests and existing web Python/Node checks in the isolated ROG worktree (22 agent + 78 web Python + 12 Node passed).
- [x] Exercise doctor and real read-only observations against the running ROG console (all 3 cameras; live streamed Qwen status->observe->3 images->Chinese report, 3 model rounds, 28.1s; mission remains IDLE).
- [x] Perform independent code review, address material findings, and report evidence boundaries (5 findings, 9 regression tests; 31 total passed locally; final ROG validation recorded below).

## Execution Record

- 2026-11-06: Existing design reused; system framework requested explicitly. API key not yet available; user instructed proceeding without it.
- 2026-11-06: ROG reboot caused temporary SSH loss. Connection recovered; isolated worktree created successfully at the stated path. Active checkout untouched.
- Publication/merge and live model verification are outside this implementation stage; keep changes reviewable and report them explicitly.
- User subsequently provided a key and explicitly authorized API testing: text, D435 image and function-call probes succeeded. The current platform uses https://maas.qianwenaiapi.com/compatible-mode/v1. The secret is only in ignored agent/.secrets/qwen.env with permissions 600; no secret enters commits.
- Stage 1 commit: 13afda2; stage 2 commit: 7c9e8be. User authorized scoped local commits after checks, without push/merge/deployment. Runtime records and secrets remain ignored.
- Ruling: External grasp/planning/control/verification/navigation/insertion services remain explicit contracts, not fabricated implementations. Current evidence only supports their unavailable state; integrating each requires its own hardware and calibration work.
- Stage 3 commit: 9a9ce4c; live read-only Qwen loop verified before commit. No control or camera service was restarted.
- Final review fixed receipt request identity and planner identity contradictions; mismatched completion/rejection retains pending. Regression tests observed failing then passing.
- Final review fixed crash-window replay and invalid tool-message ordering. Original completed/rejected receipts are cached before pending is cleared; two crash-window tests RED->GREEN.
- Final review fixed pre-action evidence reuse and unbound alignment geometry. Post-stage observation and selected-slot/camera guards verified RED->GREEN.
- Final review fixed smoke execution allowlist, retained across resume. Policy regression RED->GREEN.
- Final review replaced PID read/unlink recovery with process-lifetime OS file locks. Coordinated stale-lock race test produced two writers before fix and exactly one after; crash-release test also passed on Windows.
- No minor findings deferred. Final scope remains source and local commits in the independent worktree; no push, merge or active-service deployment.
- Final verification: 31/31 Agent tests passed on both Windows and ROG. Post-fix streamed Qwen smoke completed status->three-camera capture->Chinese report, run run-20261106T105110Z-080f970d, IDLE and motion disabled. Existing isolated web checks passed 78 Python +12 Node. Secrets absent from tracked task sources; private key file mode600. Original active main checkout's six dirty tracked files were preserved; camera/control PIDs unchanged.
- Ruling: Keep branch and development worktree in place without presenting merge/push options, following the user's explicit no-push/no-merge/no-deployment instruction. Integration requires a later user request.
