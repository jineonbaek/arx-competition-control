import copy,unittest
import cv2,numpy as np
from agent.cabinet_consensus import fuse_cabinet_frames

def frames():
    out=[]
    for i,dx in enumerate([-1,0,1,0,0]):
        rgb=np.full((240,320,3),220,np.uint8);cv2.rectangle(rgb,(70+dx,60),(250+dx,180),(230,0,0),9)
        out.append(dict(id=str(i),camera_id='d435',camera_serial='test',calibration_sha256='a'*64,intrinsics='same',
            body_configuration='same',base_from_color=np.eye(4),joints=np.zeros(6),lift_rad=5.,image_time=1.+i*.1,rgb=rgb))
    return out

class ConsensusTests(unittest.TestCase):
    def test_real_snapshot_rate_uses_middle_measurement_time_not_latest_image_time(self):
        source=frames()
        for i,f in enumerate(source):f['image_time']=1.+i*4/15
        d,last=fuse_cabinet_frames(source,now=source[-1]['image_time'])
        self.assertAlmostEqual(d['consensus_time'],1.+8/15)
        with self.assertRaises(ValueError):fuse_cabinet_frames(source,now=source[-1]['image_time']+.4)
    def test_independent_stationary_exposures_form_median_frame_without_reusing_ids(self):
        source=frames();d,last=fuse_cabinet_frames(source,now=1.4)
        np.testing.assert_allclose(np.array(d['inner_quad_px']).mean(0),[160.,120.],atol=.3)
        self.assertEqual(d['consensus_exposures'],[str(i) for i in range(5)])
        self.assertIs(last,source[-1]);self.assertLessEqual(d['quad_spread_px'],1.5)
    def test_repeats_motion_source_change_and_moving_marker_are_rejected(self):
        for kind in ['repeat','joints','calibration','marker']:
            source=frames()
            if kind=='repeat':source[-1]['image_time']=source[-2]['image_time']
            elif kind=='joints':source[-1]['joints'][1]=.02
            elif kind=='calibration':source[-1]['calibration_sha256']='b'*64
            else:source[-1]['rgb']=np.roll(source[-1]['rgb'],8,axis=1)
            with self.subTest(kind=kind),self.assertRaises(ValueError):fuse_cabinet_frames(source,now=1.4)
