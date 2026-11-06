"""Benchmark the official saved RGB-D example. No robot/camera/ROS integrations."""
import argparse
from datetime import datetime, timezone, timedelta
import importlib.util
import json
import math
import os
from pathlib import Path
import statistics
import sys
import threading
import time

ROOT = Path(__file__).resolve().parent


def summarize(values):
    if not values or any(not math.isfinite(v) or v <= 0 for v in values):
        raise ValueError("Latency samples must be finite and positive")
    ordered = sorted(values)
    rank = .95 * (len(ordered)-1)
    low = math.floor(rank)
    high = math.ceil(rank)
    p95 = ordered[low] + (ordered[high]-ordered[low]) * (rank-low)
    return {"count": len(values), "mean_ms": statistics.mean(values),
            "median_ms": statistics.median(values), "p95_ms": p95,
            "min_ms": min(values), "max_ms": max(values)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=50)
    args = parser.parse_args()
    if not 5 <= args.iterations <= 1000:
        parser.error("--iterations must be between 5 and 1000")
    import numpy as np
    import torch
    import open3d
    import pynvml

    torch.set_num_threads(4)
    torch.set_num_interop_threads(1)
    if not torch.cuda.is_available():
        raise RuntimeError("A working CUDA GPU is required for this benchmark")
    np.random.seed(20261106)
    torch.manual_seed(20261106)
    pynvml.nvmlInit()
    handle = pynvml.nvmlDeviceGetHandleByIndex(0)
    baseline = pynvml.nvmlDeviceGetMemoryInfo(handle).used
    memory_samples = []
    process_memory_samples = []
    memory_errors = []
    stop = threading.Event()

    def sample_memory():
        while not stop.is_set():
            try:
                memory_samples.append(pynvml.nvmlDeviceGetMemoryInfo(handle).used)
                for process in pynvml.nvmlDeviceGetComputeRunningProcesses(handle):
                    if process.pid == os.getpid() and process.usedGpuMemory < 8151 * 1024**2:
                        process_memory_samples.append(process.usedGpuMemory)
            except pynvml.NVMLError as exc:
                memory_errors.append(str(exc))
                return
            stop.wait(.01)

    thread = threading.Thread(target=sample_memory, daemon=True)
    thread.start()
    repo = ROOT / "upstream/graspnet-baseline"
    checkpoint = ROOT / "models/checkpoint-rs.tar"
    previous_argv = sys.argv
    sys.argv = ["demo.py", "--checkpoint_path", str(checkpoint)]
    spec = importlib.util.spec_from_file_location("official_graspnet_demo", repo / "demo.py")
    official = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(official)
    finally:
        sys.argv = previous_argv

    def timed(function):
        torch.cuda.synchronize()
        begin = time.perf_counter()
        value = function()
        torch.cuda.synchronize()
        return value, (time.perf_counter()-begin)*1000

    try:
        torch.cuda.reset_peak_memory_stats()
        net, load_ms = timed(official.get_net)
        data, prep_ms = timed(lambda: official.get_and_process_data(str(repo / "doc/example_data")))
        inputs, cloud = data
        first, cold_ms = timed(lambda: official.get_grasps(net, dict(inputs)))
        for _ in range(5):
            official.get_grasps(net, dict(inputs))
        torch.cuda.synchronize()

        inference_times = []
        for _ in range(args.iterations):
            result, duration = timed(lambda: official.get_grasps(net, dict(inputs)))
            inference_times.append(duration)

        forward_times = []
        for _ in range(args.iterations):
            with torch.no_grad():
                prediction, duration = timed(lambda: net(dict(inputs)))
            forward_times.append(duration)
            del prediction

        pipeline_times = []
        preparation_times = []
        pipeline_inference_times = []
        collision_times = []
        nms_times = []
        for _ in range(10):
            torch.cuda.synchronize()
            begin = time.perf_counter()
            (batch, scene), duration = timed(
                lambda: official.get_and_process_data(str(repo / "doc/example_data")))
            preparation_times.append(duration)
            grasps, duration = timed(lambda: official.get_grasps(net, dict(batch)))
            pipeline_inference_times.append(duration)
            raw_count = len(grasps)
            grasps, duration = timed(
                lambda: official.collision_detection(grasps, np.asarray(scene.points)))
            collision_times.append(duration)
            collision_count = len(grasps)
            grasps, duration = timed(lambda: grasps.nms().sort_by_score())
            nms_times.append(duration)
            pipeline_times.append((time.perf_counter()-begin)*1000)
        if not len(grasps) or not np.isfinite(grasps.grasp_group_array).all():
            raise RuntimeError("No finite grasp candidates survived the official pipeline")
        rotations = grasps.grasp_group_array[:, 4:13].reshape(-1, 3, 3)
        if not np.allclose(rotations @ rotations.transpose(0, 2, 1), np.eye(3), atol=1e-4):
            raise RuntimeError("Predicted grasp rotations are not orthonormal")
        output = ROOT / "runtime/benchmark"
        output.mkdir(parents=True, exist_ok=True)
        np.save(output / "grasps.npy", grasps[:50].grasp_group_array)
        report = {
            "measured_at": datetime.now(timezone(timedelta(hours=8))).isoformat(),
            "input": {"source": "official doc/example_data", "batch_size": 1,
                      "sampled_points": int(inputs["point_clouds"].shape[1]),
                      "scene_points": len(cloud.points), "num_view": 300,
                      "precision": str(next(net.parameters()).dtype),
                      "warmup_iterations": 5, "cpu_threads": 4,
                      "collision_threshold": .01, "voxel_size_m": .01},
            "environment": {"gpu": torch.cuda.get_device_name(0),
                            "compute_capability": list(torch.cuda.get_device_capability(0)),
                            "python": sys.version.split()[0], "torch": torch.__version__,
                            "torch_cuda": torch.version.cuda, "numpy": np.__version__,
                            "open3d": open3d.__version__, "model_parameters": sum(p.numel() for p in net.parameters())},
            "cold": {"model_load_ms": load_ms, "prepare_ms": prep_ms,
                     "first_inference_ms": cold_ms},
            "latency": {"forward_only": summarize(forward_times),
                        "inference_including_decode_and_cpu_copy": summarize(inference_times),
                        "offline_pipeline": summarize(pipeline_times),
                        "pipeline_preparation": summarize(preparation_times),
                        "pipeline_inference": summarize(pipeline_inference_times),
                        "pipeline_collision_filter": summarize(collision_times),
                        "pipeline_nms": summarize(nms_times)},
            "memory": {"baseline_before_model_total_gpu_mib": baseline/1024**2,
                       "sampled_process_gpu_peak_mib": max(process_memory_samples)/1024**2 if process_memory_samples else None,
                       "torch_peak_allocated_mib": torch.cuda.max_memory_allocated()/1024**2,
                       "torch_peak_reserved_mib": torch.cuda.max_memory_reserved()/1024**2,
                       "sampled_total_gpu_peak_mib": max(memory_samples)/1024**2,
                       "sample_interval_ms": 10, "sampling_errors": memory_errors},
            "candidates": {"raw": raw_count, "after_collision": collision_count,
                           "after_nms": len(grasps), "saved_for_viewing": min(50,len(grasps))},
            "scope": {"includes_live_camera": False, "includes_model_load_in_warm_latency": False,
                      "includes_gui_in_warm_latency": False, "robot_commands_sent": 0,
                      "pipeline_includes": ["RGB-D file loading", "point cloud construction and sampling",
                                           "inference and decoding", "CPU result copy", "collision filter", "NMS"]},
            "sources": json.loads((ROOT / "sources.json").read_text()),
        }
        stop.set()
        thread.join()
        (ROOT / "benchmark-results.json").write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps(report, indent=2))
    finally:
        stop.set()
        thread.join()
        pynvml.nvmlShutdown()


if __name__ == "__main__":
    main()
