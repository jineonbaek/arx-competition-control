import time,unittest
from unittest.mock import patch
import numpy as np
from insertion_observation import measure_insertion

class MeasurementTests(unittest.TestCase):
    def test_signed_depth_comes_from_battery_edge_and_mouth_not_tcp(self):
        tool=np.eye(4);tool[:3,3]=[.3,0.,.5]
        cabinet=dict(inside_axis_base=[1.,0.,0.],up_base=[0.,0.,1.],height_anchor_base_m=[.5,0.,.5])
        f=dict(camera_id='d405',id='a',image_time=time.time(),calibration_sha256='a'*64)
        front=dict(point_base_m=[.475,0.,.5],support_pixels=80,quad_px=[[0,0],[1,0],[1,1],[0,1]])
        with patch('insertion_observation.cabinet_observation',return_value=cabinet),patch('insertion_observation.battery_front',return_value=front) as edge:
            first=measure_insertion(f,tool)
            self.assertAlmostEqual(first['depth_mm'],-25.)
            self.assertEqual(edge.call_args.kwargs['outside_range_mm'],(-210.,300.))
        tool[0,3]+=.175;front['point_base_m'][0]+=.175;f['id']='b'
        with patch('insertion_observation.battery_front',return_value=front):
            after=measure_insertion(f,tool,first)
            self.assertAlmostEqual(after['depth_mm'],150.)
            front['point_base_m'][1]=.04
            with self.assertRaisesRegex(ValueError,'不一致'):measure_insertion(f,tool,first)

    def test_disappearance_is_not_insertion_success(self):
        with patch('insertion_observation.cabinet_observation',side_effect=ValueError('no red frame')):
            with self.assertRaises(ValueError):measure_insertion({'camera_id':'d405'},np.eye(4))

if __name__=='__main__':unittest.main()
