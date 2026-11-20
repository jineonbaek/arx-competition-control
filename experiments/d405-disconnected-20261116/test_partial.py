import unittest,numpy as np,cv2
from d405_partial import estimate_vertical_alignment
class PartialRimTest(unittest.TestCase):
    def frame(self):
        H,W=480,640;rgb=np.full((H,W,3),230,np.uint8)
        cv2.rectangle(rgb,(200,0),(220,49),(255,0,0),-1);cv2.rectangle(rgb,(420,0),(440,49),(255,0,0),-1);cv2.rectangle(rgb,(200,35),(440,52),(255,0,0),-1)
        cv2.rectangle(rgb,(190,85),(370,460),(25,25,25),-1)
        xx,yy=np.meshgrid(np.arange(W),np.arange(H));z=np.full_like(xx,.2,dtype=float);xyz=np.stack(((xx-320)*z/400,(yy-240)*z/400,z),axis=2);T=np.eye(4);T[:3,:3]=[[0,0,1],[0,-1,0],[1,0,0]]
        return dict(camera_id="d405",rgb=rgb,xyz=xyz,base_from_color=T)
    def test_top_clipped_rim_and_separate_front(self):
        r=estimate_vertical_alignment(self.frame());self.assertEqual(r["status"],"visual_guidance_only");self.assertFalse(r["executable"]);self.assertLessEqual(abs(r["step_mm"]),3.)
    def test_two_disconnected_clipped_red_sides(self):
        f=self.frame();f["rgb"][35:53,222:420]=230
        result=estimate_vertical_alignment(f)
        self.assertEqual(result["status"],"visual_guidance_only");self.assertFalse(result["executable"])
    def test_missing_one_side_rejected(self):
        f=self.frame();f["rgb"][:60,400:]=230
        with self.assertRaises(ValueError):estimate_vertical_alignment(f)
if __name__=="__main__":unittest.main()
