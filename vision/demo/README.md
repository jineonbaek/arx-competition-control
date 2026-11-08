# Competition vision worktree

This directory owns an isolated, read-only perception experiment on branch `codex/vision-demo`.
The source is based on main `5e67196`. Existing camera capture and robot services remain external.

Models: Grounding DINO tiny and SAM 2.1 small, using the pinned Hugging Face revisions
in sources.json through Transformers. This uses the ROG's existing CUDA 12.8 PyTorch
installation in a separate venv with system-site-packages; setup does not upgrade its packages.

Run `python3 vision/demo/setup.py`. If upstream downloads stall, use
`--endpoint https://hf-mirror.com`. Both endpoints request the same pinned revisions.
Weights, caches, environments and observations are ignored by Git.
The smoke check loads both checkpoints together and performs actual CUDA inference.

Baseline: agent tests 31 passed. The fresh worktree's broader web suite currently
reports 13 errors from absent untracked robot model assets. Vision changes must be
verified independently; no baseline failure is treated as passing.

## Perception loop

After setup, run from the worktree root:

```bash
vision/demo/.venv/bin/python vision/demo/pipeline.py --camera d405 --attempts 2
vision/demo/.venv/bin/python vision/demo/pipeline.py --camera d405 --attempts 1 --force-fallback
vision/demo/.venv/bin/python vision/demo/pipeline.py --image vision/config/d435-reference.jpg --attempts 1
```

The first command reads live published frames from the existing 9983 camera owner.
It never opens camera devices or enables depth. With --image, the input is explicitly
an offline frame and attempts must be 1. Use --reference PATH for another object;
the default crops the first configured D435 battery reference using its saved box.

Stages are local DINO proposals, structured Qwen candidate selection, Qwen coarse
box/point recovery if candidates are absent or rejected, SAM2 segmentation,
local mask/depth diagnostics, and Qwen visual review. Rejected observations can
trigger at most three new captures. --force-fallback explicitly tests the coarse
path even when DINO found candidates, and is recorded in the result.

Private configuration is read literally from the existing main checkout's
agent/.secrets/qwen.env (permissions 600). It is never copied into the worktree,
printed or committed. Model-selected pixels/IDs are validated before segmentation.

Each ignored runtime/run-*/observation-* stores rgb.png, camera.json (live mode),
robot-state.json (live mode), candidates.png, mask.png, overlay.png and result.json.
camera.json refers to the original published frame; robot-state.json is a later
read, not a guaranteed synchronized joint-state sample. Qwen visual reports and
their request durations are saved per run. Images are sent to the configured cloud
Qwen API. Model reasoning text is not written.

perception_accepted means mask/identity review passed. The result always has
robot_geometry_ready=false. With available aligned uint16 depth, diagnostic camera
surface statistics are reported; projection currently uses the pinhole approximation.
No CAD registration, distortion correction, robot-coordinate transform, collision
check, robot motion or insertion is performed by this experiment.

Run deterministic boundary/branch checks using:

```bash
cd vision/demo
.venv/bin/python -m unittest test_pipeline -v
```
