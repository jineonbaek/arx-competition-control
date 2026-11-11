# Automatic battery grasp preview integration

Goal: reuse AgentRuntime / ToolRegistry / RunStore and existing recognition, FoundationPose and point-cloud code. Preview only; TCP pending. No robot commands, deployment, merge or push.

1. Frozen scene and model interface: copy reviewed tool-policy extension; validate base_link transforms, immutable frame IDs, masks; let FoundationPose accept the recognition mask. Tests: malformed/mismatched masks and geometry rejected; existing worker tests.
2. Sequential preview tool: capture D405 using existing owner (only on explicit UI request), recognition DINO/Qwen/SAM2, FoundationPose, CAD grip references, depth diagnostics and fixed-YZ +X task-space line. Keep wrist targets null pending TCP; no IK/collision certification. Tests: rejected recognition stops; no recapture; cancellation terminates child; no physical tools; replay on saved RGB-D.
3. Secondary web preview: use existing arxgent session/worker/policy and Three.js, show stages/images/cloud/line and blocked TCP state. Test HTTP isolation, input validation and browser rendering. Commit explicit paths after diff and related tests.

Scope: D405 fine-grasp chain first. D435 recognition remains available through existing vision page; coarse lift/base motion is deliberately not connected. Snapshot-time lift transform is nominal. No model weights or calibration datasets committed. Teammate TCP worktree untouched.
