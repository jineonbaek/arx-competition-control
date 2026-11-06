"""Run the upstream official demo; save its 3D display data over SSH."""
import json
import os
from pathlib import Path
import runpy
import sys
import time

ROOT = Path(__file__).resolve().parent


def main():
    import torch
    import open3d as o3d
    show = "--show" in sys.argv[1:]
    if set(sys.argv[1:]) - {"--show"}:
        raise SystemExit("Usage: bash grasp/run-demo.sh [--show]")
    checkpoint = ROOT / "models/checkpoint-rs.tar"
    if not checkpoint.is_file():
        raise SystemExit("Missing checkpoint; run bash grasp/setup.sh first")
    output = ROOT / "runtime/official-demo"
    output.mkdir(parents=True, exist_ok=True)
    original_viewer = o3d.visualization.draw_geometries

    def save(geometries, *args, **kwargs):
        if len(geometries) < 2:
            raise RuntimeError("Official example did not produce any grasp candidates")
        if not o3d.io.write_point_cloud(str(output / "scene.ply"), geometries[0]):
            raise RuntimeError("Cannot save the scene point cloud")
        mesh = o3d.geometry.TriangleMesh()
        for item in geometries[1:]:
            mesh += item
        if not o3d.io.write_triangle_mesh(str(output / "candidate-grippers.ply"), mesh):
            raise RuntimeError("Cannot save candidate gripper meshes")
        result = {"candidate_meshes": len(geometries)-1, "source": "upstream demo.py",
                  "robot_commands_sent": 0}
        (output / "result.json").write_text(json.dumps(result, indent=2)+"\n")
        print("OFFICIAL DEMO PASS", json.dumps(result))
        if show:
            original_viewer(geometries, *args, **kwargs)

    o3d.visualization.draw_geometries = save
    repo = ROOT / "upstream/graspnet-baseline"
    os.chdir(repo)
    sys.argv = ["demo.py", "--checkpoint_path", str(checkpoint)]
    started = time.perf_counter()
    runpy.run_path(str(repo / "demo.py"), run_name="__main__")
    torch.cuda.synchronize()
    print("Cold demo elapsed seconds:", round(time.perf_counter()-started, 3))


if __name__ == "__main__":
    main()
