import unittest
import tempfile
from pathlib import Path
import numpy as np
try:
    from mesh_filter import voxelize_triangles, MeshFilter, urdf_geometry
except ImportError:
    voxelize_triangles = MeshFilter = urdf_geometry = None

def cube(origin, size):
    v=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],float)*size+origin
    faces=[[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
    return v[faces]

class MeshTests(unittest.TestCase):
    def test_solid_and_padding_preserve_gap(self):
        self.assertIsNotNone(voxelize_triangles, 'mesh filtering not implemented')
        a=voxelize_triangles(cube([0,0,0],.10),.005)
        b=voxelize_triangles(cube([.3,0,0],.1),.005)
        f=MeshFilter(np.vstack([a,b]),np.empty((0,3)),np.eye(4),.005)
        pts=np.array([[.05,.05,.05],[.2,.05,.05],[.11,.05,.05],[.15,.05,.05]])
        np.testing.assert_array_equal(f.mask(pts,np.eye(4),np.eye(4),.02),[True,False,True,False])

    def test_head_moves_independently_and_sensor_coordinates_transform(self):
        self.assertIsNotNone(MeshFilter)
        head=voxelize_triangles(cube([0,0,1],.1),.005)
        nominal=np.eye(4);nominal[2,3]=1
        f=MeshFilter(np.empty((0,3)),head,nominal,.005)
        actual=nominal.copy();actual[0,3]=.2
        sensor=np.eye(4);sensor[:3,:3]=np.diag([1,-1,-1]);sensor[2,3]=.3
        world=np.array([[.05,.05,1.05],[.25,.05,1.05]])
        pts=(world-sensor[:3,3])@sensor[:3,:3]
        np.testing.assert_array_equal(f.mask(pts,sensor,actual,.005),[False,True])

    def test_urdf_chain_visual_scale_and_ground_origin(self):
        self.assertIsNotNone(urdf_geometry)
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'meshes').mkdir();(root/'urdf').mkdir()
            triangles=cube([0,0,0],100.)
            dt=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]);a=np.zeros(len(triangles),dt);a['v']=triangles
            (root/'meshes/cube.STL').write_bytes(bytes(80)+len(a).to_bytes(4,'little')+a.tobytes())
            xml='''<robot name="t"><link name="base_link"/><link name="head_113_link"><visual><origin xyz="0 0 0.1"/><geometry><mesh filename="package://lift2/meshes/cube.STL" scale=".001 .001 .001"/></geometry></visual></link><joint name="h" type="fixed"><parent link="base_link"/><child link="head_113_link"/><origin xyz="0.2 0 1" rpy="0 0 1.5707963267948966"/></joint></robot>'''
            p=root/'urdf/test.urdf';p.write_text(xml)
            parts=urdf_geometry(p,origin_in_base=[0,0,-.168])
            self.assertEqual(len(parts),1);self.assertTrue(parts[0]['head'])
            v=parts[0]['triangles'].reshape(-1,3)
            np.testing.assert_allclose(v.min(0),[.1,0,1.268],atol=1e-7)
            np.testing.assert_allclose(v.max(0),[.2,.1,1.368],atol=1e-7)

if __name__=='__main__': unittest.main()
