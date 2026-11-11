"""Offline FoundationPose registration/tracking; reads files and saves results only."""
import argparse
import json
import time
from pathlib import Path
import numpy as np


def main():
    root=Path(__file__).resolve().parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene",type=Path,default=root/"demo_data/mustard0")
    parser.add_argument("--mesh",type=Path)
    parser.add_argument("--frames",type=int,default=5)
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    if args.frames<1:parser.error("--frames must be positive")
    mesh_path=args.mesh or args.scene/"mesh/textured_simple.obj"
    if not mesh_path.is_file():parser.error("Missing mesh: "+str(mesh_path))
    out=args.output or root/"runtime"/time.strftime("demo-%Y%m%d-%H%M%S")
    # A new directory avoids overwriting previous test evidence.
    out.mkdir(parents=True,exist_ok=False)
    from estimater import FoundationPose,ScorePredictor,PoseRefinePredictor,dr,trimesh,draw_posed_3d_box,draw_xyz_axis
    from datareader import YcbineoatReader
    import torch
    import imageio.v2 as imageio
    torch.manual_seed(0)
    np.random.seed(0)
    mesh=trimesh.load(mesh_path)
    if not np.isfinite(mesh.vertices).all():raise ValueError("Non-finite mesh")
    origin,extents=trimesh.bounds.oriented_bounds(mesh)
    bbox=np.stack([-extents/2,extents/2])
    reader=YcbineoatReader(video_dir=str(args.scene),shorter_side=None,zfar=np.inf)
    if not reader.color_files:raise ValueError("Empty RGB-D sequence")
    torch.cuda.reset_peak_memory_stats()
    est=FoundationPose(model_pts=mesh.vertices,model_normals=mesh.vertex_normals,mesh=mesh,
        scorer=ScorePredictor(),refiner=PoseRefinePredictor(),glctx=dr.RasterizeCudaContext(),
        debug_dir=str(out/"debug"),debug=0)
    results=[]
    for i in range(min(args.frames,len(reader.color_files))):
        rgb,depth=reader.get_color(i),reader.get_depth(i)
        torch.cuda.synchronize();start=time.perf_counter()
        if i==0:
            pose=est.register(K=reader.K,rgb=rgb,depth=depth,ob_mask=reader.get_mask(0).astype(bool),iteration=5)
        else:
            pose=est.track_one(rgb=rgb,depth=depth,K=reader.K,iteration=2)
        torch.cuda.synchronize();elapsed=time.perf_counter()-start
        if pose.shape!=(4,4) or not np.isfinite(pose).all():raise ValueError("Invalid pose")
        if not np.allclose(pose[:3,:3].T@pose[:3,:3],np.eye(3),atol=1e-3):raise ValueError("Invalid rotation")
        np.savetxt(out/f"pose-{i:04d}.txt",pose)
        centered=pose@np.linalg.inv(origin)
        vis=draw_posed_3d_box(reader.K,img=rgb,ob_in_cam=centered,bbox=bbox)
        vis=draw_xyz_axis(vis,ob_in_cam=centered,scale=.1,K=reader.K,thickness=3,transparency=0,is_input_rgb=True)
        imageio.imwrite(out/f"overlay-{i:04d}.png",vis)
        results.append(dict(frame=i,id=reader.id_strs[i],seconds=elapsed,T_camera_from_object=pose.tolist()))
        print(f"frame {i}: {elapsed:.3f}s",flush=True)
    report=dict(scene=str(args.scene),mesh=str(mesh_path),frame="camera optical",translation_unit="metres",
        torch=torch.__version__,cuda=torch.version.cuda,gpu=torch.cuda.get_device_name(),
        peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20,frames=results)
    (out/"result.json").write_text(json.dumps(report,indent=2,allow_nan=False))
    print(str(out/"result.json"),flush=True)


if __name__=="__main__":
    main()
