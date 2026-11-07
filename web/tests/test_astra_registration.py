import ctypes
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from astra_registration import FactoryParams,factory_parameters,load_calibration,Registration


def example(size=(4,3)):
    return dict(serial='synthetic',depth_size=list(size),color_size=list(size),
                depth_K=[[2,0,1],[0,2,1],[0,0,1]],color_K=[[2,0,1],[0,2,1],[0,0,1]],
                depth_D=[0]*5,color_D=[0]*5,depth_to_color_R=np.eye(3).tolist(),depth_to_color_translation_m=[0,0,0])

class RegistrationTests(unittest.TestCase):
    def load(self,data):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'calibration.json';path.write_text(json.dumps(data))
            return load_calibration(path,'synthetic',tuple(data['depth_size']))
    def test_all_ff_factory_data_is_rejected_without_exporting_nan(self):
        params=FactoryParams.from_buffer_copy(bytes([255])*120)
        report=factory_parameters(Mock(get_property=Mock(return_value=params)))
        self.assertEqual(ctypes.sizeof(FactoryParams),120)
        self.assertEqual(report['status'],'invalid')
        self.assertNotIn('parameters',report)
        json.dumps(report,allow_nan=False)
    def test_calibration_rejects_wrong_identity_resolution_nan_and_invalid_rigid_transform(self):
        changes=[{'serial':'another'},{'color_size':[8,6]},{'depth_K':[[float('nan'),0,1],[0,2,1],[0,0,1]]},
                 {'depth_to_color_R':[[1,0,0],[0,1,0],[0,0,-1]]},{'depth_to_color_translation_m':[100,0,0]}]
        for change in changes:
            with self.assertRaises(ValueError):self.load({**example(),**change})
    def test_factory_rejects_finite_but_invalid_rotation(self):
        params=FactoryParams()
        params.l_intr_p[:]=[500,500,320,240];params.r_intr_p[:]=[500,500,320,240]
        self.assertEqual(factory_parameters(Mock(get_property=Mock(return_value=params)))['status'],'invalid')

    def test_factory_read_failure_is_a_diagnostic_not_a_depth_failure(self):
        report=factory_parameters(Mock(get_property=Mock(side_effect=RuntimeError('unsupported property'))))
        self.assertEqual(report['status'],'unavailable')
        self.assertIn('unsupported property',report['reason'])

    def test_identity_preserves_depth_and_invalid_zero_pixels(self):
        raw=np.array([[0,1000,2000,0],[500,0,800,900],[0,0,3000,1000]],np.uint16)
        np.testing.assert_array_equal(Registration(self.load(example())).apply(raw),raw)
    def test_occlusion_keeps_nearest_color_optical_depth(self):
        data=example();data['color_K']=[[.1,0,1],[0,.1,1],[0,0,1]]
        raw=np.zeros((3,4),np.uint16);raw[1,1]=2000;raw[1,2]=1000
        result=Registration(self.load(data)).apply(raw)
        self.assertEqual(result[1,1],1000)

    def test_registered_depth_uses_color_optical_z_in_metres(self):
        data=example();data['depth_to_color_translation_m']=[0,0,.1]
        raw=np.zeros((3,4),np.uint16);raw[1,1]=1000
        result=Registration(self.load(data)).apply(raw)
        self.assertEqual(result[1,1],1100)
    def test_negative_transformed_depth_is_invalid(self):
        data=example();data['depth_to_color_translation_m']=[0,0,-.2]
        raw=np.full((3,4),100,np.uint16)
        self.assertFalse(Registration(self.load(data)).apply(raw).any())

class CaptureRegistrationTests(unittest.TestCase):
    def capture(self):
        from types import SimpleNamespace
        from astra_depth import AstraDepth
        capture=AstraDepth.__new__(AstraDepth)
        capture.device=Mock();capture.serial='synthetic';capture.intrinsics={'source':'estimated'}
        modes=SimpleNamespace(ONI_IMAGE_REGISTRATION_OFF=0,ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR=1)
        api=SimpleNamespace(OniImageRegistrationMode=modes)
        return capture,api

    def test_invalid_factory_data_keeps_raw_depth_available(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        import astra_depth
        capture,api=self.capture();capture.device.get_image_registration_mode.return_value=SimpleNamespace(value=0)
        with tempfile.TemporaryDirectory() as tmp,patch.object(astra_depth,'ROOT',Path(tmp)),patch.object(astra_depth,'factory_parameters',return_value={'status':'invalid'}):
            capture.configure_registration(api)
        self.assertFalse(capture.aligned_to_color)
        self.assertEqual(capture.calibration['status'],'required')
        self.assertIsNone(capture.projector)
        capture.device.set_image_registration_mode.assert_called_once_with(0)

    def test_measured_calibration_is_loaded_only_for_this_serial_and_profile(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        import astra_depth
        capture,api=self.capture();capture.device.get_image_registration_mode.return_value=SimpleNamespace(value=0)
        with tempfile.TemporaryDirectory() as tmp,patch.object(astra_depth,'ROOT',Path(tmp)),patch.object(astra_depth,'factory_parameters',return_value={'status':'invalid'}):
            directory=Path(tmp)/'vision/config';directory.mkdir(parents=True)
            data=example((640,480));data['depth_K']=data['color_K']=[[500,0,320],[0,500,240],[0,0,1]]
            path=directory/'astra-synthetic.json';path.write_text(json.dumps(data))
            capture.configure_registration(api)
            self.assertTrue(capture.aligned_to_color)
            self.assertEqual(capture.registration,'measured_depth_to_color')
            self.assertEqual(capture.calibration['validation'],'pending_physical_check')
            data['serial']='different';path.write_text(json.dumps(data))
            capture.configure_registration(api)
            self.assertFalse(capture.aligned_to_color)
            self.assertEqual(capture.calibration['status'],'error')

    def test_factory_registration_requires_sdk_readback(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        import astra_depth
        factory={'status':'valid','sha256':'synthetic','parameters':{'r_intr_p':[500,500,320,240],'r_k':[1,2,3,4,5]}}
        for accepted in [True,False]:
            with self.subTest(accepted=accepted):
                capture,api=self.capture()
                capture.device.get_image_registration_mode.side_effect=[SimpleNamespace(value=n) for n in ([0,1] if accepted else [0,0,0])]
                with tempfile.TemporaryDirectory() as tmp,patch.object(astra_depth,'ROOT',Path(tmp)),patch.object(astra_depth,'factory_parameters',return_value=factory):
                    capture.configure_registration(api)
                self.assertEqual(capture.aligned_to_color,accepted)
                self.assertEqual(capture.calibration['status'],'loaded' if accepted else 'error')
                if accepted:self.assertEqual(capture.intrinsics['coeffs'],[1,2,4,5,3])

if __name__=='__main__':unittest.main()
