# GraspNet official-demo evaluation on ROG

Scope: install the official graspnet/graspnet-baseline in an isolated environment,
run its provided RGB-D example with official RealSense weights, and measure GPU
memory and inference latency. No robot, CAN, ROS or live camera integration.

Base checkpoint: f21b2f2 (active competition main, already saved in Git).
Development branch: codex/graspnet-demo; active main and navigation edits stay intact.

## Reproducible stages

1. Freeze upstream revisions and install dependencies / compiled CUDA operators.
2. Run the official example and save point-cloud / grasp-candidate artifacts.
3. Benchmark warmed inference and complete pipeline separately, including GPU memory.
4. Record validation, exact commands, compatibility patches and measured limits.

Stage 1 validated: CUDA operators compile for sm_120, 5 real functional checks
pass, and the upstream official RGB-D example produces 50 displayed candidates.
The demo window is captured as PLY artifacts when run over SSH.

## Run on this ROG

    cd /home/arx/arx-graspnet-demo
    bash grasp/setup.sh
    bash grasp/run-demo.sh

The installation reuses the existing ACT environment's PyTorch 2.10.0+cu128 and
CUDA 12.8 compiler without changing that environment. Additional packages and
compiled operators live in grasp/.venv. Setup uses the Tsinghua PyPI mirror.
NumPy is pinned to 1.26.4 to avoid old API alias/ABI incompatibilities.

Outputs: grasp/runtime/official-demo/{scene.ply,candidate-grippers.ply,result.json}.
For a desktop 3D window, run bash grasp/run-demo.sh --show in a graphical session.
This viewer displays candidates and has no robot-execution callback.

The apply_compatibility.py helper changes the removed torch._six.container_abcs import to
Python's collections.abc. The neural network and grasp decoding are unmodified.
The CUDA compiler needs both ACT bin and nvvm/bin on PATH, plus CUDA_HOME pointing
to its targets/x86_64-linux directory.

## Checkpoint provenance

The original author Google Drive link returned a download-quota error. The
downloaded checkpoint is publicly redistributed by dgrachev/a2_pretrained on
Hugging Face at a pinned revision. The same bytes were verified through
hf-mirror.com on this ROG (12,468,415 bytes; approximately 11.89 MiB).
See sources.json for URLs, revision and SHA256. Identity against the author's
currently rate-limited download has not been independently verified.
Checkpoint loading uses PyTorch weights_only=True.

Additional dependencies occupy approximately 1.6 GiB; existing shared PyTorch
and the ACT CUDA toolkit are not included in that size. No model files are
committed.

## Sources

- https://github.com/graspnet/graspnet-baseline
- https://github.com/graspnet/graspnetAPI
- Official RealSense checkpoint, linked from upstream README:
  https://drive.google.com/file/d/1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk/view

Dependencies, vendor code, model weights, build outputs and runtime logs are ignored.
Only this task's source, instructions and summarized measurements are committed.
