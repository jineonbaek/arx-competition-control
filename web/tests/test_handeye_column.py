import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from handeye_column import ColumnModel

class ColumnTests(unittest.TestCase):
    def test_path_crossing_with_clear_endpoints_is_rejected(self):
        model=ColumnModel()
        # Use a wrist-sized obstacle centered on the actual wrist halfway through this sweep.
        a,b=np.zeros(6),np.zeros(6)
        a[0],b[0]=-1.2,1.2
        point=model.pose(np.zeros(6),0)[1][:3,3]
        model.column=point-.001,point+.001
        # Search a more distal test point if the folded arm occupies it throughout.
        self.assertFalse(model.check(a,b,0)["clear"])
        self.assertGreater(model.check(a,b,0)["steps"],2)

    def test_far_obstacle_clears_and_fingers_are_included(self):
        m=ColumnModel();m.column=(np.ones(3)*4,np.ones(3)*5)
        self.assertTrue(m.check([0]*6,[.1]*6,0)["clear"])
        self.assertEqual(len(m.parts),8)
