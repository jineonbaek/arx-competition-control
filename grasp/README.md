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


## Measured official-example benchmark

Measured on the ROG RTX 5070 Laptop (8,151 MiB), Python 3.12.12,
PyTorch 2.10.0+cu128 / CUDA 12.8; FP32, batch 1, 20,000 sampled points,
300 views, 4 CPU threads. The saved official scene has 513,688 valid points.
Five warmups precede 50 measured inference runs; the full offline pipeline
is measured 10 times. CUDA is synchronized around every timed region.

| Measurement | Median | P95 |
| --- | ---: | ---: |
| Network forward only | 69.87 ms | 70.02 ms |
| Inference + decode + result CPU copy | 70.55 ms | 70.73 ms |
| RGB-D files to collision-filtered / NMS grasps | 496.88 ms | 510.24 ms |

Peak PyTorch allocated memory: 662.28 MiB.
Peak PyTorch reserved memory: 1028.00 MiB.
NVML compute-process sampled peak: 1194.00 MiB.
NVML whole-GPU sampled peak: 1661.50 MiB.
These are different accounting scopes, not values to add together. NVML samples
every 10 ms and can miss shorter peaks; PyTorch reports its allocator high-water mark.

The final sampled cloud produced 332 raw candidates,
112 after collision filtering and
71 after NMS; 50 are saved for viewing.
Candidate arrays are checked for finite values and orthonormal rotations.

The approximately 0.50-second offline pipeline includes disk reads, depth-to-cloud
conversion/sampling, inference, collision filtering and NMS. It excludes model
loading, live capture, network transport, GUI rendering and any robot planning
or motion. This is one saved example, not battery-grasp accuracy or a live-camera FPS claim.
The model has 1,025,964 trainable parameters.
See benchmark-results.json for full measured values and checkpoint provenance.

Repeat:

    cd /home/arx/arx-graspnet-demo
    OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 MKL_NUM_THREADS=4 grasp/.venv/bin/python grasp/benchmark.py --iterations 50

The repeat overwrites the local benchmark-results.json measurement report; review
its diff before committing a new measurement. Runtime logs and 3D artifacts remain ignored.
Only run-demo.sh --show opens an interactive viewer; it never executes a robot command.

Validation: 5 installation/CUDA/checkpoint checks and 3 benchmark-statistics tests pass.
The setup script was rerun successfully, and the upstream official demo produced 50 candidates.
