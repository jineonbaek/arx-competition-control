import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from handeye_column import ColumnModel


class ModelSourceTests(unittest.TestCase):
    def test_model_identity_matches_loaded_bytes_and_mesh_cache_tracks_edits(self):
        with tempfile.TemporaryDirectory() as t:
            package=Path(t);(package/'urdf').mkdir();(package/'meshes').mkdir();mesh=package/'meshes/shape.STL'
            dtype=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('extra','<u2')]);data=np.zeros(4,dtype)
            data['vertices']=[[[0,0,0],[.1,0,0],[0,.1,0]],[[0,0,0],[.1,0,0],[0,0,.1]],[[0,0,0],[0,.1,0],[0,0,.1]],[[.1,0,0],[0,.1,0],[0,0,.1]]]
            def save():mesh.write_bytes(bytes(80)+np.array([4],'<u4').tobytes()+data.tobytes())
            save();links=['base_link']+['right_link'+str(i) for i in range(21,29)]
            xml='<robot><joint name="joint4"><origin xyz="0 0 0" rpy="0 0 0"/></joint>'
            for name in links:xml+='<link name="'+name+'"><visual><geometry><mesh filename="package://lift2/meshes/shape.STL"/></geometry></visual></link>'
            for i in range(21,29):xml+='<joint name="right_joint'+str(i)+'"><origin xyz="0 0 .1" rpy="0 0 0"/><axis xyz="0 0 1"/><limit lower="0" upper=".01"/></joint>'
            urdf=package/'urdf/lift2_with_sensors.urdf';urdf.write_text(xml+'</robot>')
            a=ColumnModel(package);old=a.column[0].copy()
            self.assertEqual(a.source_hashes[str(mesh.resolve())],hashlib.sha256(mesh.read_bytes()).hexdigest())
            self.assertEqual(a.source_hashes[str(urdf.resolve())],hashlib.sha256(urdf.read_bytes()).hexdigest())
            data['vertices']+=.2;save();b=ColumnModel(package)
            np.testing.assert_allclose(b.column[0]-old,[.2,.2,.2],atol=1e-7)
            self.assertNotEqual(a.source_hashes[str(mesh.resolve())],b.source_hashes[str(mesh.resolve())])
