"""Offline URDF visual-mesh self filter; metre units, separate flexible head.

5 mm surface voxels and enclosed-volume fill are an approximation of the STL,
not a replacement for calibrated articulation or a deformable column model.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.ndimage import binary_fill_holes
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def origin_transform(node):
    T = np.eye(4)
    if node is not None:
        T[:3, 3] = np.fromstring(node.get('xyz', '0 0 0'), sep=' ')
        T[:3, :3] = Rotation.from_euler('xyz', np.fromstring(node.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return T


def urdf_geometry(urdf, origin_in_base=(-.015273, .000010, -.168)):
    """Load visual triangles at q=0, with all scale and frame conversions applied."""
    urdf = Path(urdf)
    root = ET.parse(urdf).getroot()
    transforms = {'base_link': np.eye(4)}
    transforms['base_link'][:3, 3] = -np.asarray(origin_in_base)
    head = {'base_link': False}
    pending = list(root.findall('joint'))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            if parent not in transforms:
                continue
            transforms[child] = transforms[parent] @ origin_transform(joint.find('origin'))
            head[child] = head[parent] or child == 'head_113_link'
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError('URDF disconnected tree or cycle')
    parts = []
    for link in root.findall('link'):
        name = link.get('name')
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                raise ValueError('Only explicit triangle visual meshes supported')
            filename = mesh.get('filename')
            if not filename.startswith('package://lift2/'):
                raise ValueError('Unexpected mesh package')
            path = urdf.parent.parent / filename.removeprefix('package://lift2/')
            data = path.read_bytes()
            count = int.from_bytes(data[80:84], 'little')
            if len(data) != 84 + 50 * count:
                raise ValueError('Expected binary STL: ' + str(path))
            dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
            tri = np.frombuffer(data, dtype, count=count, offset=84)['vertices'].astype(float)
            tri *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            T = transforms[name] @ origin_transform(visual.find('origin'))
            tri = tri @ T[:3, :3].T + T[:3, 3]
            if not np.isfinite(tri).all():
                raise ValueError('Non-finite geometry')
            parts.append(dict(link=name, head=head[name], triangles=tri, source=str(path),
                              sha256=hashlib.sha256(data).hexdigest()))
    return parts


def voxelize_triangles(triangles, pitch=.005):
    """Rasterize subdivided surface (max edge pitch/2), then fill enclosed cells.

    Vertices are rounded to voxel centers. The query expands distance by
    pitch/2 + sqrt(3)*pitch/2 to cover sampling plus center quantization.
    Thin openings below this resolution can close; unclosed shells may not fill.
    """
    triangles = np.asarray(triangles, float)
    if triangles.size == 0:
        return np.empty((0, 3))
    lower = np.floor(triangles.min(axis=(0, 1)) / pitch).astype(int) - 2
    upper = np.ceil(triangles.max(axis=(0, 1)) / pitch).astype(int) + 2
    shape = upper - lower + 1
    if np.prod(shape) > 100_000_000:
        raise ValueError('Mesh extent too large: check units')
    grid = np.zeros(shape, bool)
    stack = [triangles[i:i+4096] for i in range(0, len(triangles), 4096)]
    while stack:
        tri = stack.pop()
        edge = np.maximum.reduce([np.linalg.norm(tri[:, i] - tri[:, (i+1) % 3], axis=1) for i in range(3)])
        small = tri[edge <= pitch * .5]
        if len(small):
            index = np.rint(small.reshape(-1, 3) / pitch).astype(int) - lower
            grid[tuple(index.T)] = True
        large = tri[edge > pitch * .5]
        if len(large):
            a, b, c = large[:, 0], large[:, 1], large[:, 2]
            ab, bc, ca = (a+b)*.5, (b+c)*.5, (c+a)*.5
            for corners in [(a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)]:
                child = np.stack(corners, axis=1)
                stack.extend(child[i:i+4096] for i in range(0, len(child), 4096))
    grid = binary_fill_holes(grid)
    return (np.argwhere(grid) + lower) * pitch


class MeshFilter:
    def __init__(self, rigid, head, nominal_head, pitch):
        self.rigid = cKDTree(rigid) if len(rigid) else None
        self.head = cKDTree(head) if len(head) else None
        self.nominal_head = np.asarray(nominal_head)
        self.pitch = float(pitch)
        self.approximation_margin = self.pitch * (.5 + np.sqrt(3) * .5)

    @classmethod
    def load(cls, path):
        with np.load(path) as z:
            return cls(z['rigid'], z['head'], z['nominal_head'], float(z['pitch']))

    def mask(self, points, sensor_in_body, head_in_body, padding=.02):
        if padding < 0:
            raise ValueError('Negative padding')
        p = np.asarray(points) @ sensor_in_body[:3, :3].T + sensor_in_body[:3, 3]
        result = np.zeros(len(p), bool)
        good = np.isfinite(p).all(axis=1)
        radius = padding + self.approximation_margin
        if self.rigid is not None:
            result[good] |= self.rigid.query(p[good], distance_upper_bound=radius, workers=1)[0] <= radius
        if self.head is not None:
            # Move points back into the nominal head-assembly location.
            back = self.nominal_head @ np.linalg.inv(head_in_body)
            q = p[good] @ back[:3, :3].T + back[:3, 3]
            result[good] |= self.head.query(q, distance_upper_bound=radius, workers=1)[0] <= radius
        return result


def build(urdf, cad_file, output):
    cad = json.loads(Path(cad_file).read_text())
    parts = urdf_geometry(urdf, cad['frame']['origin_in_urdf_base_link_m'])
    groups = {False: [], True: []}
    audit = []
    for part in parts:
        points = voxelize_triangles(part['triangles'])
        groups[part['head']].append(points)
        row = {k: v for k, v in part.items() if k != 'triangles'}
        row.update(triangles=len(part['triangles']), occupied_voxels=len(points),
                   bounds_m=[part['triangles'].min(axis=(0, 1)).tolist(), part['triangles'].max(axis=(0, 1)).tolist()])
        audit.append(row)
        print(part['link'], row['triangles'], len(points), flush=True)
    head_pose = np.eye(4)
    head_pose[:3, :3] = np.diag([1, -1, -1])
    head_pose[:3, 3] = cad['top']['imu_origin_m']
    rigid, head = [np.unique(np.vstack(groups[g]), axis=0) for g in (False, True)]
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, rigid=rigid, head=head, nominal_head=head_pose, pitch=.005)
    metadata = dict(urdf_sha256=hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),
                    pitch_m=.005, query_extra_margin_m=.005*(.5+np.sqrt(3)*.5),
                    pose='all URDF joints zero; user confirmed matching recorded assembly',
                    head_motion='head_113 subtree moves with candidate head IMU; column remains nominal',
                    rigid_voxels=len(rigid), head_voxels=len(head), parts=audit,
                    limitations=['visual meshes, not validated collision solids',
                                 'enclosed voxel fill; sub-5mm openings can close; open shells may not fill',
                                 'no joint states in replay; all arm/lift joints fixed at delivery zero pose',
                                 'column distributed bending not modeled'])
    output.with_suffix('.json').write_text(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('urdf'); p.add_argument('cad'); p.add_argument('output')
    a = p.parse_args()
    build(a.urdf, a.cad, a.output)
