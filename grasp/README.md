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

Stage 0: scope saved; model installation and measurement not yet performed.

## Sources

- https://github.com/graspnet/graspnet-baseline
- https://github.com/graspnet/graspnetAPI
- Official RealSense checkpoint, linked from upstream README:
  https://drive.google.com/file/d/1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk/view

Dependencies, vendor code, model weights, build outputs and runtime logs are ignored.
Only this task's source, instructions and summarized measurements are committed.
