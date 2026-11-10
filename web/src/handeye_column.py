"""Conservative right-arm vs central column clearance in the DIY URDF's base frame."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

PACKAGE = Path(__file__).resolve().parents[1] / "public/models/lift2/diy-20261105"
MARGIN = .020
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


class ColumnModel:
    def __init__(self, package=PACKAGE, *, side="right"):
        xml = ET.parse(package/"urdf/lift2_with_sensors.urdf")
        self.lift = origin(xml.find("joint[@name='joint4']/origin"))
        def mesh(name):
            collision = xml.find(f"link[@name='{name}']/collision")
            path = collision.find("geometry/mesh").get("filename").removeprefix("package://lift2/")
            v = vertices(package/path)
            m = origin(collision.find("origin"))
            return v @ m[:3,:3].T + m[:3,3]
        base = mesh("base_link")
        # Above the chassis/wheels, the tall fixed structure is the central column.
        upper = base[base[:,2] > .09]
        lo, hi = upper.min(0), upper.max(0)
        lo[:2] -= .01
        hi[:2] += .01
        lo[2] = 0.0
        self.column = lo, hi
        self.parts = []
        first = {"left": 11, "right": 21}[side]
        self.side = side
        self.first = first
        for n in range(first, first + 8):
            j = xml.find(f"joint[@name='{side}_joint{n}']")
            m = origin(j.find("origin"))
            axis = vector(j.find("axis"),"xyz")
            v = mesh(f"{side}_link{n}")
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
            v = corners @ link[:3,:3].T + link[:3,3]
            gap = np.maximum(0, np.maximum(self.column[0]-v.max(0),v.min(0)-self.column[1]))
            closest = min(closest,(float(np.linalg.norm(gap)), f"{self.side}_link{i+self.first}"))
        return closest, wrist

    def check(self, start, end, lift):
        a,b = np.asarray(start,float),np.asarray(end,float)
        if a.shape != (6,) or b.shape != (6,) or not np.isfinite(np.r_[a,b,lift]).all():
            raise ValueError("无效关节或升降反馈")
        delta = b-a
        steps = max(1,math.ceil(np.max(np.abs(delta))/math.radians(.5)))
        # Between grid points, any point can travel at most reach * sum(|dq|).
        between = self.reach * np.abs(delta).sum() / (2*steps)
        gap,link = min(self.pose(a + t*delta,lift)[0] for t in np.linspace(0,1,steps+1))
        bound = gap-between
        return {"clear":bool(bound > MARGIN),"lower_mm":round(max(0.,bound)*1000,1),
                "margin_mm":int(MARGIN*1000),"link":link,"steps":steps+1}
