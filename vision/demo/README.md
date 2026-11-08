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
