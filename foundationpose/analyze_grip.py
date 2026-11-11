"""Measure supplied CAD and check the nominal grip by sampled closing-direction rays."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import open3d as o3d

MAIN=Path("/home/arx/arx-competition-control")
BATTERY_FROM_WRIST=np.array([[0,0,1],[0,1,0],[-1,0,0]],float)


def ray_scene(mesh):
    scene=o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh(
        o3d.core.Tensor(np.asarray(mesh.vertices,np.float32)),
        o3d.core.Tensor(np.asarray(mesh.faces,np.uint32))))
    return scene


def y_extremes(mesh,x,z):
    scene=ray_scene(mesh)
    out=[]
    for side in [-1,1]:
        origins=np.stack([x,np.full_like(x,side*100),z],-1).reshape(-1,3)
        dirs=np.zeros_like(origins);dirs[:,1]=-side
        hit=scene.cast_rays(o3d.core.Tensor(np.c_[origins,dirs].astype(np.float32)))["t_hit"].numpy()
        values=side*100-side*hit
        values[~np.isfinite(hit)]=np.nan
        out.append(values.reshape(x.shape))
    return out


def analyze():
    cad=MAIN/"web/cad/gripper-v2"
    names=["battery-print.stl","viewer-right-v2.stl","viewer-left-v2.stl"]
    battery,right,left=[trimesh.load(cad/name) for name in names]
    loops=battery.section(plane_origin=[0,0,0],plane_normal=[0,1,0]).discrete
    holes=[p for p in loops if 20<np.ptp(p[:,0])<60 and 8<np.ptp(p[:,2])<25]
    if len(holes)!=1:raise ValueError("Expected one central through-hole")
    hole=holes[0];lo,hi=hole.min(0),hole.max(0)
    front=float(battery.bounds[1,2])
    upper_faces=np.abs(battery.face_normals[:,1])>.999
    upper_faces&=battery.triangles_center[:,2]>hi[2]+.1
    planes=np.round(battery.triangles_center[upper_faces,1],3)
    areas=battery.area_faces[upper_faces]
    def dominant(sign):
        values=np.unique(planes[planes*sign>0])
        return float(max(values,key=lambda y:areas[planes==y].sum()))
    ylow,yhigh=dominant(-1),dominant(1)
    bump=battery.vertices[(battery.vertices[:,1]>yhigh+.01)&(battery.vertices[:,2]>hi[2]+.1)]
    if not len(bump):raise ValueError("Missing lower protrusion")
    bump_lo,bump_hi=bump.min(0),bump.max(0)
    floor=(np.abs(right.face_normals[:,1])>.999)&np.isclose(right.triangles_center[:,1],5,atol=.001)&(right.triangles_center[:,0]>20)
    floor_vertices=right.triangles[floor].reshape(-1,3)
    if not len(floor_vertices):raise ValueError("Missing V2 relief floor")
    notch_lo,notch_hi=floor_vertices.min(0),floor_vertices.max(0)
    # Pin the identifiable design features; do not silently infer a different tool.
    if not np.allclose(hi[[0,2]]-lo[[0,2]],[40,14],atol=.02):raise ValueError("Handle geometry changed")
    thickness=yhigh-ylow
    bump_height=float(bump_hi[1]-yhigh)
    recess=float(notch_lo[1]-right.bounds[0,1])
    notch_width=float(notch_hi[2]-notch_lo[2])
    bump_width=float(bump_hi[0]-bump_lo[0])
    exact=front-(lo[2]+hi[2])/2
    insertion=float(np.ceil(exact))
    cx=float((lo[0]+hi[0])/2)
    reference=[cx,0,float((hi[2]+front)/2)]
    profile=dict(version=1,units="mm",mesh_sha256={name:hashlib.sha256((cad/name).read_bytes()).hexdigest() for name in names},
        assumptions=["凸起朝下，右腕约定夹姿 J6 -90°；带槽夹指位于下侧","沿电池 -Z 伸入，整车对齐后对应 base_link +X","夹指安装沿用现有 URDF 和四孔拟合；毫米开口尚未实机标定"],
        points_object_mm=dict(front=[cx,0,front],grip=reference,tip=[cx,0,front-insertion]),
        approach_object=[0,0,-1],bump_direction_object=[0,1,0],required_j6_deg=-90,
        hole_bounds_mm=[lo.tolist(),hi.tolist()],hole_size_mm=[float(hi[0]-lo[0]),float(hi[2]-lo[2])],
        handle_thickness_mm=thickness,bridge_depth_mm=float(front-hi[2]),bump_height_mm=bump_height,
        bump_bounds_mm=[bump_lo.tolist(),bump_hi.tolist()],notch_width_mm=notch_width,notch_depth_mm=recess,
        bump_width_mm=bump_width,lateral_clearance_each_mm=(notch_width-bump_width)/2,
        bump_relief_clearance_mm=recess-bump_height,insertion_exact_mm=float(exact),insertion_mm=insertion,
        tip_beyond_bridge_mm=float(hi[2]-(front-insertion)),limitations="名义 CAD 候选；采样检查不含无人机、机架、线缆、制造误差或全路径碰撞")
    manifest=json.loads((MAIN/"web/models-lift2.json").read_text())
    urdf=MAIN/"web/public"/manifest["model"]["public_path"]/manifest["model"]["urdf"]
    xml=ET.parse(urdf).getroot()
    joints={}
    for link in ["right_link27","right_link28"]:
        joint=next(j for j in xml.findall("joint") if j.find("child").get("link")==link)
        joints[link]=np.fromstring(joint.find("origin").get("xyz"),sep=" ")*1000
    meta=manifest["model"]["gripper_v2"]
    shift=[np.array(meta["raw_to_finger_translation_mm"][side]) for side in ["positive_y","negative_y"]]
    offset=[joints["right_link27"]+shift[0],joints["right_link28"]+shift[1]]
    width_center=(notch_lo[2]+notch_hi[2])/2
    wrist_z=-float(np.mean([v[2]+width_center for v in offset]))
    tip_forward=offset[0][0]+right.bounds[1,0]
    profile["tip_alignment_error_mm"]=float(abs(tip_forward-(offset[1][0]+left.bounds[1,0])))
    if profile["tip_alignment_error_mm"]>.05:raise ValueError("Finger leading ends are not aligned")
    closed_y=float(np.mean([offset[0][1]+right.bounds[0,1],offset[1][1]+left.bounds[1,1]]))
    profile["nominal_mount"]=dict(urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
        raw_translations_mm=[v.tolist() for v in shift],joint_origins_mm={k:v.tolist() for k,v in joints.items()},
        note="四孔拟合沿用预览；不是实机 TCP 标定")
    q=thickness/2
    def fingers(depth,reverse=False):
        R=BATTERY_FROM_WRIST.copy()
        if reverse:R=R@np.diag([1,-1,-1])
        t=np.array([cx+(-wrist_z if reverse else wrist_z),closed_y if reverse else -closed_y,front-depth+tip_forward])
        meshes=[]
        for i,(mesh,sign) in enumerate([(right,1),(left,-1)]):
            m=mesh.copy()
            m.vertices=(m.vertices+offset[i]+[0,sign*q,0])@R.T+t
            meshes.append(m)
        return meshes
    gx,gz=np.meshgrid(np.arange(-45,40.001,.5),np.arange(150,196.001,.5))
    bmin,bmax=y_extremes(battery,gx,gz)
    checks={}
    for label,depth,reverse in [("candidate",insertion,False),("wrong_j6_sign",insertion,True),("too_deep_42mm",42.,False)]:
        results=[]
        for i,mesh in enumerate(fingers(depth,reverse)):
            fmin,fmax=y_extremes(mesh,gx,gz)
            gap=fmin-bmax if (i==0)!=reverse else bmin-fmax
            results.append(dict(min_gap_mm=float(np.nanmin(gap)),penetrating_samples=int(np.count_nonzero(gap<-.1))))
        checks[label]=results
    nominal=fingers(insertion)
    lower=ray_scene(nominal[0])
    origins=bump.copy();origins[:,1]=0
    directions=np.zeros_like(origins);directions[:,1]=1
    hit=lower.cast_rays(o3d.core.Tensor(np.c_[origins,directions].astype(np.float32)))["t_hit"].numpy()
    covered=np.isfinite(hit)&np.isclose(hit,yhigh+recess,atol=.02)
    profile["recess_coverage"]=dict(protrusion_vertex_samples=len(bump),inside_recess_samples=int(covered.sum()),
        minimum_clearance_mm=float(np.min(hit-bump[:,1])),scope="凸起表面 CAD 顶点沿闭合方向投影到夹指槽底")
    if not covered.all():raise ValueError("Protrusion is not fully covered by the intended inner recess")
    t=np.array([cx+wrist_z,-closed_y,front-insertion+tip_forward])
    raw_line=np.array([[notch_lo[0],notch_lo[1],width_center],[notch_hi[0],notch_lo[1],width_center]])
    profile["notch_floor_midline_object_mm"]=((raw_line+offset[0]+[0,q,0])@BATTERY_FROM_WRIST.T+t).tolist()
    profile["ray_check"]=dict(grid_mm=.5,penetration_tolerance_mm=.1,cases=checks,
                             scope="closing-direction surface samples, not a certified collision check")
    if any(v["penetrating_samples"] for v in checks["candidate"]):raise ValueError("Candidate penetrates CAD")
    if not any(v["penetrating_samples"] for v in checks["wrong_j6_sign"]):raise ValueError("Orientation regression not detected")
    return profile,battery,nominal


def schematic(profile,battery,fingers):
    parts=['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1040 500"><rect width="1040" height="500" fill="#0b1220"/><style>text{font-family:Arial,Microsoft YaHei,sans-serif;fill:#dce8f0;font-size:15px}.hint{fill:#93a6ba;font-size:13px}</style>']
    def text(x,y,s,cls=""):parts.append(f'<text x="{x}" y="{y}" class="{cls}">{s}</text>')
    front=profile["points_object_mm"]["front"][2]
    def uv(p):return [80+(front-p[2])*6,230+(p[0]-.15602875)*3]
    loops=battery.section(plane_origin=[0,0,0],plane_normal=[0,1,0]).discrete
    text(30,34,"提手正面 · F → T 沿电池 -Z 伸入")
    parts.append('<defs><clipPath id="front"><rect x="20" y="56" width="460" height="350"/></clipPath><clipPath id="side"><rect x="530" y="56" width="490" height="310"/></clipPath></defs><g clip-path="url(#front)">')
    for loop in loops:
        points=" ".join(f"{a:.2f},{b:.2f}" for a,b in map(uv,loop))
        parts.append(f'<polyline points="{points}" fill="none" stroke="#9eb2c8" stroke-width="2"/>')
    parts.append("</g>")
    parts.append('<rect x="20" y="56" width="460" height="350" fill="none" stroke="#263345"/>')
    for key,color,label in [("front","#ffcf65","F 前沿"),("grip","#ff9256","G 夹持中心"),("tip","#42e7ec","T 指尖平面")]:
        x,y=uv(profile["points_object_mm"][key])
        parts.append(f'<circle cx="{x}" cy="{y}" r="5" fill="{color}"/>')
        text(x-20,95 if key=="front" else 340 if key=="grip" else 375,label)
        parts.append(f'<path d="M{x},{y} L{x},{110 if key=="front" else 320 if key=="grip" else 355}" stroke="{color}" stroke-width="1"/>')
    f,t=map(uv,[profile["points_object_mm"]["front"],profile["points_object_mm"]["tip"]])
    parts.append(f'<path d="M{f[0]},{f[1]} L{t[0]},{t[1]}" stroke="#42e7ec" stroke-width="3"/>')
    text(60,433,f'越过最前沿 {profile["insertion_mm"]:.0f} mm；指尖进入孔内约 {profile["tip_beyond_bridge_mm"]:.1f} mm')
    text(530,34,"侧剖面 · 凸起朝下，带槽夹指在下")
    def side(p):return [790+(front-p[2])*3,220+p[1]*3]
    parts.append('<g clip-path="url(#side)">')
    for mesh,color in [(battery,"#7f94ab"),(fingers[0],"#42a9ca"),(fingers[1],"#c48a59")]:
        section=mesh.section(plane_origin=[profile["points_object_mm"]["grip"][0],0,0],plane_normal=[1,0,0])
        if section:
            for curve in section.discrete:
                points=" ".join(f"{a:.2f},{b:.2f}" for a,b in map(side,curve))
                parts.append(f'<polyline points="{points}" fill="{color}" fill-opacity=".25" stroke="{color}" stroke-width="1.5"/>')
    # Show the two side rails omitted by the center cut, without moving either finger.
    rail_x=profile["points_object_mm"]["grip"][0]-(profile["notch_width_mm"]/2+5)
    rail=fingers[0].section(plane_origin=[rail_x,0,0],plane_normal=[1,0,0])
    if rail:
        for curve in rail.discrete:
            points=" ".join(f"{a:.2f},{b:.2f}" for a,b in map(side,curve))
            parts.append(f'<polyline points="{points}" fill="none" stroke="#42e7ec" stroke-width="2" stroke-dasharray="6 4"/>')
    floor_line=[side(np.array(p)) for p in profile["notch_floor_midline_object_mm"]]
    parts.append(f'<line x1="{floor_line[0][0]}" y1="{floor_line[0][1]}" x2="{floor_line[1][0]}" y2="{floor_line[1][1]}" stroke="#ffcf65" stroke-width="4"/>')
    tip=profile["points_object_mm"]["tip"]
    tip_x=side(tip)[0]
    parts.append(f'<line x1="{tip_x}" x2="{tip_x}" y1="155" y2="310" stroke="#ffcf65" stroke-width="1.5" stroke-dasharray="3 3"/>')
    parts.append("</g>")
    text(555,335,"两指尖齐平；青色虚线为槽两侧的前伸齿")
    text(555,365,"金色为内侧凹槽底面，凸起落在槽内")
    text(555,390,f'让位槽 {profile["notch_depth_mm"]:.1f} mm - 凸起 {profile["bump_height_mm"]:.1f} mm = 名义间隙 {profile["bump_relief_clearance_mm"]:.1f} mm')
    text(555,420,"J6 = -90°（约定夹姿），闭合方向夹住提手厚度")
    text(30,478,"仅 CAD 几何候选；不是腕部前伸指令，未包含机架与制造/安装误差。","hint")
    parts.append("</svg>")
    return "".join(parts)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    profile,battery,fingers=analyze()
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/"battery-grip.json").write_text(json.dumps(profile,ensure_ascii=False,indent=2)+"\n")
    (args.output/"battery-grip.svg").write_text(schematic(profile,battery,fingers))
    print(json.dumps(profile,ensure_ascii=False,indent=2))


if __name__=="__main__":main()
