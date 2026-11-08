"""Isolated GPU worker. Reads a saved D405 frame; never opens devices or robot interfaces."""
import importlib.util
import json
from pathlib import Path
import sys
import time
import numpy as np


def gripper_lines(position, rotation, width, depth):
    left,right=width/2,-width/2
    base=-.020
    local=np.array([[depth,left,0],[base,left,0],[base,right,0],[depth,right,0],
                    [base,0,0],[base-.04,0,0]],float)
    return local@rotation.T+position


def main():
    import torch
    demo,frame_path,frame_id,output,*box=sys.argv[1:]
    demo=Path(demo);box=list(map(int,box));started=time.monotonic()
    if not torch.cuda.is_available():raise RuntimeError('CUDA GPU 未就绪')
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'vendor-python'))
    import pyrealsense2 as rs
    with np.load(frame_path,allow_pickle=False) as f:
        if str(f['id'])!=frame_id:raise ValueError('快照文件已更新')
        xyz=f['xyz'];rgb=f['rgb'];T=f['body4_from_color'];k=json.loads(str(f['intrinsics']))
    intr=rs.intrinsics();intr.width,intr.height=k['width'],k['height']
    intr.fx,intr.fy,intr.ppx,intr.ppy=k['fx'],k['fy'],k['cx'],k['cy']
    intr.coeffs=k['coeffs'];intr.model=getattr(rs.distortion,k['model'].split('.')[-1])
    def pixel(point):
        if point[2]<=0:return None
        return rs.rs2_project_point_to_pixel(intr,point.tolist())
    x,y,r,b=box
    # Include a border of table points so the target isn't treated as an isolated object.
    h,w=xyz.shape[:2]
    ys=slice(max(y-12,0),min(b+12,h));xs=slice(max(x-12,0),min(r+12,w))
    local=xyz[ys,xs];valid=local[:,:,2]>0
    if np.count_nonzero(xyz[y:b,x:r,2])<100:raise ValueError('选区内有效深度点不足 100 个')
    pts=local[valid];colors=rgb[ys,xs][valid]
    torch.manual_seed(0);rng=np.random.default_rng(0)
    idx=rng.choice(len(pts),20000,replace=len(pts)<20000)
    sys.argv=['demo.py','--checkpoint_path',str(demo/'models/checkpoint-rs.tar')]
    path=demo/'upstream/graspnet-baseline/demo.py'
    spec=importlib.util.spec_from_file_location('graspnet_official_demo',path)
    upstream=importlib.util.module_from_spec(spec);spec.loader.exec_module(upstream)
    net=upstream.get_net()
    batch=dict(point_clouds=torch.from_numpy(pts[idx][None]).cuda(),
               cloud_colors=colors[idx].astype(np.float32)/255.)
    group=upstream.get_grasps(net,batch)
    raw=len(group)
    accepted=[]
    for i,g in enumerate(group):
        uv=pixel(g.translation)
        if uv and x<=uv[0]<r and y<=uv[1]<b:accepted.append(i)
    group=group[np.array(accepted,dtype=int)]
    if len(group):
        group=upstream.collision_detection(group,xyz[xyz[:,:,2]>0])
    filtered=len(group)
    if len(group):group=group.nms().sort_by_score()
    candidates=[]
    for g in group[:20]:
        p=g.translation;R=g.rotation_matrix
        lines=gripper_lines(p,R,g.width,g.depth)
        candidates.append(dict(score=float(g.score),width_mm=float(g.width*1000),depth_mm=float(g.depth*1000),
             camera_m=p.tolist(),rotation_camera=R.tolist(),body4_mm=((T[:3,:3]@p+T[:3,3])*1000).tolist(),
             rotation_body4=(T[:3,:3]@R).tolist(),approach_body4=(T[:3,:3]@R[:,0]).tolist(),
             lines=lines.tolist(),pixels=[pixel(v) for v in lines]))
    result=dict(id=frame_id,roi=box,raw=raw,in_roi=len(accepted),after_collision=filtered,
                after_nms=len(group),candidates=candidates,elapsed_s=round(time.monotonic()-started,2))
    Path(output).write_text(json.dumps(result,allow_nan=False))

if __name__=='__main__':main()
