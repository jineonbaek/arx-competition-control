"""CAD mesh clearance against the fixed base, using the preview's URDF visuals."""
import math
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import fcl
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

PACKAGE = Path(__file__).resolve().parents[1] / "public/models/lift2/diy-20261105"
CORNERS = np.array([[a,b,c] for a in (0,1) for b in (0,1) for c in (0,1)])
NOMINAL_RAD_PER_M = 41.54


def vector(node, key):
    return np.fromstring(node.get(key), sep=" ")


def origin(node):
    m = np.eye(4)
    if node is not None:
        m[:3,3] = vector(node,"xyz")
        m[:3,:3] = Rotation.from_euler("xyz", vector(node,"rpy")).as_matrix()
    return m


def vertices(path):
    data = path.read_bytes()
    record = np.dtype([("normal","<f4",3),("vertices","<f4",(3,3)),("extra","<u2")])
    return np.frombuffer(data, dtype=record, offset=84)["vertices"].reshape(-1,3).astype(float)


@lru_cache(maxsize=64)
def cad_mesh(path, scale, transform):
    m=np.asarray(transform,float).reshape(4,4)
    v=vertices(path)*scale
    v=v@m[:3,:3].T+m[:3,3]
    def build(points):
        geometry=fcl.BVHModel()
        geometry.beginModel(len(points),len(points)//3)
        geometry.addSubModel(points,np.arange(len(points),dtype=np.int32).reshape(-1,3))
        geometry.endModel()
        return geometry
    if path.name=='base_link.STL':
        triangles=v.reshape(-1,3,3)
        upper=triangles[:,:,2].max(1)>.09
        # Same CAD triangles, split at chassis height so the tall base's BVH stays tight.
        geometry=tuple((p.min(0),p.max(0),fcl.CollisionObject(build(p)))
                       for p in (triangles[upper].reshape(-1,3),triangles[~upper].reshape(-1,3)) if len(p))
    else:geometry=build(v)
    return v,geometry


def mesh_gap(obstacle, geometry, transform, bounds):
    obj=fcl.CollisionObject(geometry,fcl.Transform(transform[:3,:3],transform[:3,3]))
    if isinstance(obstacle,tuple):
        pairs=[]
        for lo,hi,part in obstacle:
            delta=np.maximum(0,np.maximum(lo-bounds[1],bounds[0]-hi))
            pairs.append((float(np.linalg.norm(delta)),part))
        best=math.inf
        for bound,part in sorted(pairs,key=lambda pair:pair[0]):
            if bound>=best:continue
            distance=float(fcl.distance(part,obj,fcl.DistanceRequest(),fcl.DistanceResult()))
            if not math.isfinite(distance):raise ValueError('CAD mesh distance is not finite')
            best=min(best,max(0.,distance))
            if best==0:break
        return best
    gap=float(fcl.distance(obstacle,obj,fcl.DistanceRequest(),fcl.DistanceResult()))
    if not math.isfinite(gap):raise ValueError('CAD mesh distance is not finite')
    return max(0.,gap)


class ColumnModel:
    def __init__(self, package=PACKAGE, *, side="right"):
        xml = ET.parse(package/"urdf/lift2_with_sensors.urdf")
        self.lift = origin(xml.find("joint[@name='joint4']/origin"))
        def mesh(name):
            visual = xml.find(f"link[@name='{name}']/visual")
            node=visual.find("geometry/mesh")
            path=node.get("filename").removeprefix("package://lift2/")
            scale=tuple(np.fromstring(node.get('scale','1 1 1'),sep=' '))
            return cad_mesh(package/path,scale,tuple(origin(visual.find('origin')).ravel()))
        base,geometry = mesh("base_link")
        self.column = base.min(0),base.max(0)
        self.obstacle=geometry
        self.parts = []
        self.meshes=[]
        first = {"left": 11, "right": 21}[side]
        self.side = side
        self.first = first
        for n in range(first, first + 8):
            j = xml.find(f"joint[@name='{side}_joint{n}']")
            m = origin(j.find("origin"))
            axis = vector(j.find("axis"),"xyz")
            v,geometry = mesh(f"{side}_link{n}")
            self.meshes.append(geometry)
            if n >= first + 6:
                # Include the entire possible prismatic finger travel.
                v = np.vstack([v + axis*float(j.find("limit").get(b)) for b in ("lower","upper")])
            lo,hi = v.min(0),v.max(0)
            self.parts.append((m,axis,lo+CORNERS*(hi-lo)))
        self.reach = sum(np.linalg.norm(m[:3,3]) for m,_,_ in self.parts) + max(
            np.linalg.norm(c,axis=1).max() for _,_,c in self.parts)

    def wrist_transform(self, q, lift):
        parent = self.lift.copy()
        parent[2,3] += lift / NOMINAL_RAD_PER_M
        for angle, (m, axis, _) in zip(q, self.parts[:6]):
            parent = parent @ m
            parent[:3,:3] = parent[:3,:3] @ Rotation.from_rotvec(axis*angle).as_matrix()
        return parent

    def inverse_wrist(self, q, lift, xyz_mm, rpy_deg, limits_deg):
        """Local, bounded IK of the URDF wrist in base_link; XYZ Euler angles are RPY."""
        target = np.asarray(xyz_mm, float) / 1000
        shoulder = (self.lift @ self.parts[0][0])[:3,3].copy()
        shoulder[2] += lift / NOMINAL_RAD_PER_M
        if np.max(np.abs(target - shoulder)) > self.reach:
            raise ValueError("目标在机械臂可达范围之外")
        orientation = Rotation.from_euler("xyz", rpy_deg, degrees=True)
        lo, hi = np.radians(limits_deg).T.copy()
        if self.side == "right":
            lo[0] = max(lo[0], math.radians(-35.8))
        q = np.asarray(q, float)
        if np.any(q < lo) or np.any(q > hi):
            raise ValueError("当前起姿超出规划关节范围")

        def residual(angles):
            wrist = self.wrist_transform(angles, lift)
            difference = orientation.inv() * Rotation.from_matrix(wrist[:3,:3])
            return np.r_[wrist[:3,3] - target, difference.as_rotvec()]

        fit = least_squares(residual, q, bounds=(lo, hi), max_nfev=100,
                            ftol=1e-11, xtol=1e-11, gtol=1e-11)
        error = fit.fun
        if (not fit.success or not np.isfinite(error).all()
                or np.linalg.norm(error[:3]) > .0005
                or np.linalg.norm(error[3:]) > math.radians(.2)):
            raise ValueError("当前姿态和关节限位内未找到足够精确的逆解")
        if not self.check(q, fit.x, lift)["clear"]:
            raise ValueError("腕部目标路径的 CAD 几何与底座/立柱相交或无法分离")
        return fit.x.tolist()

    def pose(self, q, lift):
        parent = self.lift.copy()
        parent[2,3] += lift/NOMINAL_RAD_PER_M
        closest = (math.inf, "")
        wrist = None
        for i,(m,axis,corners) in enumerate(self.parts):
            link = (parent if i < 6 else wrist) @ m
            if i < 6:
                link[:3,:3] = link[:3,:3] @ Rotation.from_rotvec(axis*q[i]).as_matrix()
                parent = link
            if i == 5:
                wrist = link
            def gap(offset):
                shifted=link.copy();shifted[:3,3]+=link[:3,:3]@axis*offset
                box=corners@shifted[:3,:3].T+shifted[:3,3]
                return mesh_gap(self.obstacle,self.meshes[i],shifted,(box.min(0),box.max(0)))
            distance=gap(0.)
            if i>=6:
                # Certify the full possible jaw stroke without replacing the finger with a box.
                def stroke(a,b,ga,gb,depth=0):
                    lower=min(ga,gb)-(b-a)/2
                    if lower>0 or min(ga,gb)<=0:return max(0.,lower)
                    if depth>=20:return 0.
                    middle=(a+b)/2;gm=gap(middle)
                    return min(stroke(a,middle,ga,gm,depth+1),stroke(middle,b,gm,gb,depth+1))
                distance=stroke(0.,.044,distance,gap(.044))
            closest = min(closest,(distance, f"{self.side}_link{i+self.first}"))
        return closest, wrist

    def check(self, start, end, lift):
        a,b = np.asarray(start,float),np.asarray(end,float)
        if a.shape != (6,) or b.shape != (6,) or not np.isfinite(np.r_[a,b,lift]).all():
            raise ValueError("无效关节或升降反馈")
        delta = b-a
        steps = max(1,math.ceil(np.max(np.abs(delta))/math.radians(.5)))
        samples={}
        def sample(t):
            if t not in samples:samples[t]=self.pose(a+t*delta,lift)[0]
            return samples[t]
        def interval(left,right,depth=0):
            gl,ll=sample(left);gr,lr=sample(right)
            gap,link=min((gl,ll),(gr,lr))
            # Mesh distance is Lipschitz; refine unresolved intervals instead of applying 20 mm.
            lower=gap-self.reach*np.abs(delta).sum()*(right-left)/2
            if lower>0 or gap<=0:return max(0.,lower),link
            if depth>=20:return 0.,link
            mid=(left+right)/2
            first=interval(left,mid,depth+1)
            if first[0]<=0:return first
            return min(first,interval(mid,right,depth+1))
        lower,link=math.inf,''
        for left,right in zip(np.linspace(0,1,steps+1)[:-1],np.linspace(0,1,steps+1)[1:]):
            lower,link=min((lower,link),interval(float(left),float(right)))
            if lower<=0:break
        return {"clear":bool(lower>0),"lower_mm":round(max(0.,lower)*1000,1),
                "margin_mm":0,"link":link,"steps":len(samples),"geometry":"urdf_visual_mesh"}
