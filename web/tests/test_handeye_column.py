import sys,unittest
from pathlib import Path
import numpy as np
import fcl
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from handeye_column import ColumnModel, PACKAGE, vertices

class ColumnTests(unittest.TestCase):
    def test_path_crossing_with_clear_endpoints_is_rejected(self):
        model=ColumnModel()
        # Use a wrist-sized obstacle centered on the actual wrist halfway through this sweep.
        a,b=np.zeros(6),np.zeros(6)
        a[0],b[0]=-1.2,1.2
        wrist=model.wrist_transform(np.zeros(6),0)
        local=vertices(PACKAGE/'meshes/right_link26.STL')[0]
        point=wrist[:3,:3]@local+wrist[:3,3]
        model.obstacle=fcl.CollisionObject(fcl.Box(.002,.002,.002),fcl.Transform(point))
        self.assertGreater(model.pose(a,0)[0][0],.02)
        self.assertGreater(model.pose(b,0)[0][0],.02)
        self.assertFalse(model.check(a,b,0)["clear"])
        self.assertGreater(model.check(a,b,0)["steps"],2)

    def test_far_obstacle_clears_and_fingers_are_included(self):
        m=ColumnModel();m.obstacle=fcl.CollisionObject(fcl.Box(1,1,1),fcl.Transform([4.5,4.5,4.5]))
        self.assertTrue(m.check([0]*6,[.1]*6,0)["clear"])
        self.assertEqual(len(m.parts),8)

    def test_recorded_left_offset_is_clear_in_actual_cad(self):
        m=ColumnModel()
        a=np.radians([24.85910378894652,73.50516584357881,56.88885966654423,
                      22.99405525011277,23.78129103037266,-93.83171729412882])
        b=np.radians([24.6913787037265,73.90528853639248,57.21765252861165,
                      23.057212474597932,23.61460264308277,-93.81138314817196])
        self.assertGreater(m.pose(b,5.104333400726318)[0][0],.1)
        result=m.check(a,b,5.104333400726318)
        self.assertTrue(result['clear'])
        self.assertEqual(result['margin_mm'],0)
        self.assertEqual(result['geometry'],'urdf_visual_mesh')

    def test_clear_mesh_gap_below_twenty_mm_is_allowed(self):
        m=ColumnModel()
        # Isolate distance decision from assets; no fixed clearance threshold is permitted.
        actual=m.pose
        m.pose=lambda q,h:((.012,'right_link23'),actual(q,h)[1])
        result=m.check(np.zeros(6),np.zeros(6),0)
        self.assertTrue(result['clear'])
        self.assertAlmostEqual(result['lower_mm'],12.)
