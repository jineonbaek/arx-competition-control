"""One frozen D405 frame, one battery pose. No devices or robot commands."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent


def image_url(rgb):
    ok,data=cv2.imencode(".jpg",rgb[...,::-1])
    if not ok:raise ValueError("Image encoding failed")
    return "data:image/jpeg;base64,"+base64.b64encode(data).decode()


def segment(rgb,roi):
    h,w=rgb.shape[:2]
    x,y,r,b=roi
    if not (0<=x<r<=w and 0<=y<b<=h and min(r-x,b-y)>=12):raise ValueError("Invalid battery ROI")
    mask=np.zeros((h,w),np.uint8)
    # Rectangle initializes GrabCut; the full rectangle is never treated as a precise silhouette.
    cv2.grabCut(rgb,mask,(x,y,r-x,b-y),np.zeros((1,65)),np.zeros((1,65)),5,cv2.GC_INIT_WITH_RECT)
    fg=(mask==cv2.GC_FGD)|(mask==cv2.GC_PR_FGD)
    if fg.sum()<100:raise ValueError("电池轮廓不足，请框住完整电池并留少量背景")
    return fg


def input_mask(rgb, roi, supplied=None):
    if supplied is None:
        return segment(rgb, roi)
    if supplied.ndim!=2 or supplied.shape!=rgb.shape[:2] or not np.isfinite(supplied).all():
        raise ValueError("Recognition mask must match the frozen raw color image")
    fg=supplied>0
    y,x=np.nonzero(fg)
    if len(x)<100 or not (0<=roi[0]<roi[2]<=rgb.shape[1] and 0<=roi[1]<roi[3]<=rgb.shape[0]):
        raise ValueError("Empty mask or invalid ROI")
    if x.min()<roi[0] or x.max()>=roi[2] or y.min()<roi[1] or y.max()>=roi[3]:
        raise ValueError("Mask does not belong to this ROI")
    return fg


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame",type=Path,required=True)
    parser.add_argument("--id",required=True)
    parser.add_argument("--roi",type=int,nargs=4,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--mask",type=Path,help="Reviewed mask in raw image pixels; skip GrabCut")
    parser.add_argument("--mesh",type=Path,default=ROOT.parent/"web/cad/gripper-v2/battery-print.stl")
    parser.add_argument("--mesh-scale",type=float,default=.001)
    args=parser.parse_args()
    if not np.isfinite(args.mesh_scale) or args.mesh_scale<=0:parser.error("Invalid mesh scale")
    started=time.perf_counter()
    with np.load(args.frame,allow_pickle=False) as f:
        if str(f["id"])!=args.id:raise ValueError("快照已更新")
        rgb=f["rgb"];depth=f["raw_depth"].astype(np.float32)*float(f["scale"])
        k=json.loads(str(f["intrinsics"]));base=f["base_from_color"]
    if rgb.shape[:2]!=depth.shape or not np.isfinite(base).all():raise ValueError("Invalid RGB-D geometry")
    supplied=cv2.imread(str(args.mask),cv2.IMREAD_GRAYSCALE) if args.mask else None
    if args.mask and supplied is None:raise ValueError("Cannot read recognition mask")
    mask=input_mask(rgb,args.roi,supplied)
    if np.count_nonzero(mask&(depth>.07)&(depth<1.5))<100:raise ValueError("电池有效深度不足")
    mask_vis=rgb.copy()
    mask_vis[mask]=(mask_vis[mask]*.6+np.array([0,210,255])*.4).astype(np.uint8)
    sys.path.insert(0,"/home/arx/arx-competition-control/vision/vendor-python")
    import pyrealsense2 as rs
    intr=rs.intrinsics()
    intr.width,intr.height=k["width"],k["height"]
    intr.fx,intr.fy,intr.ppx,intr.ppy=k["fx"],k["fy"],k["cx"],k["cy"]
    intr.model=getattr(rs.distortion,k["model"].split(".")[-1]);intr.coeffs=k["coeffs"]
    h,w=depth.shape
    # The network renders a pinhole camera. Rectify RGB, aligned depth and mask together.
    rays=[rs.rs2_project_point_to_pixel(intr,[(x-k["cx"])/k["fx"],(y-k["cy"])/k["fy"],1.])
          for y in range(h) for x in range(w)]
    maps=np.asarray(rays,np.float32).reshape(h,w,2)
    color=cv2.remap(rgb,maps[...,0],maps[...,1],cv2.INTER_LINEAR)
    z=cv2.remap(depth,maps[...,0],maps[...,1],cv2.INTER_NEAREST)
    fg=cv2.remap(mask.astype(np.uint8),maps[...,0],maps[...,1],cv2.INTER_NEAREST).astype(bool)
    K=np.array([[k["fx"],0,k["cx"]],[0,k["fy"],k["cy"]],[0,0,1]],float)
    from estimater import FoundationPose,ScorePredictor,PoseRefinePredictor,dr,trimesh,nvdiffrast_render
    from scipy.spatial.transform import Rotation
    import torch
    torch.manual_seed(0);np.random.seed(0)
    source=args.mesh
    mesh=trimesh.load(source)
    mesh.apply_scale(args.mesh_scale)
    torch.cuda.reset_peak_memory_stats()
    output=args.output.parent
    est=FoundationPose(model_pts=mesh.vertices,model_normals=mesh.vertex_normals,mesh=mesh,
        scorer=ScorePredictor(),refiner=PoseRefinePredictor(),glctx=dr.RasterizeCudaContext(),
        debug_dir=str(output/"debug"),debug=0)
    torch.cuda.synchronize();tick=time.perf_counter()
    pose=est.register(K=K,rgb=color,depth=z,ob_mask=fg,iteration=5)
    torch.cuda.synchronize();seconds=time.perf_counter()-tick
    if pose.shape!=(4,4) or not np.isfinite(pose).all():raise ValueError("Invalid estimated pose")
    if not np.allclose(pose[:3,:3].T@pose[:3,:3],np.eye(3),atol=1e-3) or np.linalg.det(pose[:3,:3])<.99:
        raise ValueError("Invalid estimated rotation")
    transform=base@pose
    center=mesh.bounds.mean(axis=0)
    center_cam=pose[:3,:3]@center+pose[:3,3]
    center_base=transform[:3,:3]@center+transform[:3,3]
    def project(points):
        pts=np.asarray(points)@pose[:3,:3].T+pose[:3,3]
        if np.any(pts[:,2]<=0):raise ValueError("Estimated CAD is behind camera")
        return np.rint([rs.rs2_project_point_to_pixel(intr,p.tolist()) for p in pts]).astype(int)
    corners=trimesh.bounds.corners(mesh.bounds)
    pixels=project(corners)
    vis=rgb.copy()
    edges=[]
    for i in range(8):
        for j in range(i+1,8):
            if np.count_nonzero(corners[i]!=corners[j])==1:
                cv2.line(vis,tuple(pixels[i]),tuple(pixels[j]),(40,255,90),2,cv2.LINE_AA)
                edges.append([pixels[i].tolist(),pixels[j].tolist()])
    _,render_depth,_=nvdiffrast_render(K=K,H=h,W=w,mesh=mesh,
        ob_in_cams=torch.as_tensor(pose[None],dtype=torch.float32,device="cuda"),glctx=est.glctx)
    silhouette=(render_depth[0].detach().cpu().numpy()>0).astype(np.uint8)
    contours,_=cv2.findContours(silhouette,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        uv=contour[:,0,:]
        distorted=np.rint(maps[uv[:,1],uv[:,0]]).astype(np.int32).reshape(-1,1,2)
        cv2.polylines(vis,[distorted],True,(0,220,255),2,cv2.LINE_AA)
    axes=project(np.vstack([center,center+np.eye(3)*.06]))
    for i,c in enumerate([(255,70,70),(70,255,70),(70,140,255)],1):
        cv2.line(vis,tuple(axes[0]),tuple(axes[i]),c,3,cv2.LINE_AA)
    grip=None
    profile_path=ROOT.parent/"web/public/cad/battery-grip.json"
    source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
    if profile_path.is_file():
        profile=json.loads(profile_path.read_text())
        if source_hash==profile["mesh_sha256"]["battery-print.stl"] and np.isclose(args.mesh_scale,.001):
            from battery_grip import draw_reference
            grip=draw_reference(profile,transform,pose,project,vis)
    report=dict(id=args.id,roi=args.roi,status="complete",frame="base_link",translation_unit="mm",
        T_camera_from_object=pose.tolist(),T_base_from_object=transform.tolist(),
        center_base_mm=(center_base*1000).tolist(),center_camera_mm=(center_cam*1000).tolist(),
        rpy_base_deg=Rotation.from_matrix(transform[:3,:3]).as_euler("xyz",degrees=True).tolist(),
        model_size_mm=(mesh.extents*1000).tolist(),mask_pixels=int(mask.sum()),
        mesh_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),registration_s=seconds,
        total_s=time.perf_counter()-started,peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20,
        overlay=image_url(vis),mask_image=image_url(mask_vis),edges=edges,grasp_reference=grip,
        message="仅位姿预览；请核对轮廓和 CAD 叠加，未生成运动指令")
    args.output.write_text(json.dumps(report,allow_nan=False))
    print("result",str(args.output),flush=True)


if __name__=="__main__":main()
