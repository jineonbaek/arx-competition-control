from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from agent.tests.test_d405_aperture_alignment import frame
from d405_placement_preview import D405PlacementPreview


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        files={'config/right-grip-tcp.json':{'T_wrist_from_tool':np.eye(4).tolist()},
            'config/battery-envelope.json':{'bounds_mm':[[-40,-35,0],[40,35,195]]},
            'web/public/cad/battery-grip.json':{'points_object_mm':{'tip':[0,0,169]}},
            'config/taught-placement.json':{'d405_target_offset_mm':42.4,'d405_step_limit_mm':3.}}
        for name,value in files.items():
            path=self.root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value))
        data=frame();data.update(joints=np.zeros(6),lift_rad=np.array(0.))
        self.cache=self.root/'cache';self.cache.mkdir();np.savez(self.cache/'frame.npz',**data)
        class Model:
            def wrist_transform(self,*args):return np.eye(4)
        class Controller:
            models={'right':Model()}
            def snapshot(inner):return dict(command_epoch=0,chassis_active=False,body={'height':0.},arms={'right':{'joints':[0.]*6}})
        class Capture:
            cache=self.cache
            count=0
            def action(inner,request):
                inner.count+=1;value=dict(data,id=np.array('frame-'+str(inner.count)))
                np.savez(inner.cache/'frame.npz',**value)
                return {'id':str(value['id'])}
        self.capture=Capture();self.preview=D405PlacementPreview(Controller(),self.capture,self.root)

    def test_missing_battery_still_publishes_real_rim_without_motion_api(self):
        with patch('d405_placement_preview.measure_left_edges',side_effect=ValueError('battery front missing')):
            result=self.preview.action({'op':'refresh'})
        self.assertEqual(result['status'],'partial');self.assertEqual(result['reason'],'battery front missing')
        self.assertEqual(result['rim']['status'],'visible_partial');self.assertIsNone(result['step_mm'])
        self.assertEqual(result['battery_front']['status'],'unavailable')
        self.assertTrue((self.root/'web/public/camera-overlay/d405-horizontal.jpg').exists())

    def test_replaced_capture_is_rejected_before_publishing(self):
        self.capture.action=lambda data:{'id':'another-frame'}
        with self.assertRaisesRegex(ValueError,'快照'):self.preview.action({'op':'refresh'})
        self.assertFalse((self.root/'web/public/camera-overlay/d405-horizontal.jpg').exists())


if __name__=='__main__':unittest.main()
