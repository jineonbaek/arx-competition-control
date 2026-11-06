import json
from pathlib import Path
import sys
import tempfile
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from camera_depth import write_depth

class DepthTests(unittest.TestCase):
    def test_lossless_depth_and_metadata_use_the_same_immutable_frame(self):
        raw=np.array([[0,1,1000],[2000,65535,40]],dtype=np.uint16)
        with tempfile.TemporaryDirectory() as tmp:
            meta,files=write_depth(Path(tmp),"d405","abc123-7",raw,.0001,
                {"fx":100.,"fy":101.,"cx":1.,"cy":.5},True,7,1234.)
            restored=cv2.imread(str(Path(tmp)/meta["image_file"]),cv2.IMREAD_UNCHANGED)
            np.testing.assert_array_equal(restored,raw)
            self.assertEqual(restored.dtype,np.uint16)
            self.assertEqual(meta["scale_m_per_unit"],.0001)
            self.assertEqual(meta["width"],3)
            self.assertEqual(meta["height"],2)
            self.assertEqual(meta["frame_number"],7)
            self.assertEqual(meta["sensor_timestamp_ms"],1234.)
            self.assertTrue(meta["aligned_to_color"])
            self.assertAlmostEqual(meta["valid_fraction"],5/6)
            self.assertEqual(len(files),2)
            self.assertTrue(all(p.is_file() for p in files))
            json.dumps(meta,allow_nan=False)
    def test_absent_usb_camera_is_detected_without_sdk_discovery(self):
        from battery_rgbd import usb_camera_present
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            self.assertFalse(usb_camera_present('D435 product',root))
            device=root/'4-2';device.mkdir();(device/'product').write_text('D435 product\n')
            self.assertTrue(usb_camera_present('D435 product',root))
            self.assertFalse(usb_camera_present('D405 product',root))
            (root/'4-3').mkdir()
            self.assertFalse(usb_camera_present('absent',root))

    def test_rejects_invalid_depth_without_writing_a_frame(self):
        with tempfile.TemporaryDirectory() as tmp:
            for raw,scale in [(np.zeros((2,2),np.float32),.001),(np.zeros((2,2),np.uint16),0),(np.zeros((2,2),np.uint16),float("nan"))]:
                with self.assertRaises(ValueError):
                    write_depth(Path(tmp),"astra","abc123-1",raw,scale,{},False,1,0.)
            self.assertEqual(list(Path(tmp).iterdir()),[])

class AstraTests(unittest.TestCase):
    def test_openni_abi_patch_is_idempotent_and_rejects_unknown_bindings(self):
        import importlib.util
        path=Path(__file__).resolve().parents[2]/"vision/scripts/setup-depth.py"
        spec=importlib.util.spec_from_file_location("depth_setup",path)
        setup=importlib.util.module_from_spec(spec);spec.loader.exec_module(setup)
        source="OniDeviceInfo._fields_ = [\n    ('uri', (ctypes.c_char * 256)),\n    ('vendor', (ctypes.c_char * 256)),\n    ('name', (ctypes.c_char * 256)),\n    ('usbVendorId', ctypes.c_ushort),\n]\n"
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/"binding.py";p.write_text(source)
            setup.patch_binding(p);first=p.read_text();setup.patch_binding(p)
            self.assertEqual(first,p.read_text())
            self.assertIn("('serialNumber', (ctypes.c_char * 256))",first)
            p.write_text("unknown")
            with self.assertRaises(RuntimeError):setup.patch_binding(p)

    def test_unpatched_abi_is_rejected_before_sdk_initialization(self):
        import ctypes
        from types import SimpleNamespace
        from unittest.mock import Mock,patch
        from astra_depth import AstraDepth
        sdk=Mock();fake=SimpleNamespace(openni2=sdk,_openni2=SimpleNamespace(OniDeviceInfo=ctypes.c_int))
        with patch.dict(sys.modules,{'openni':fake}):
            with self.assertRaisesRegex(RuntimeError,'Unsafe OpenNI ABI'):AstraDepth()
        sdk.initialize.assert_not_called()

    def test_astra_copies_depth_buffer_and_never_claims_color_registration(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from astra_depth import AstraDepth
        buffer=bytearray(np.array([0,1000,2000,3000],np.uint16).tobytes())
        frame=SimpleNamespace(width=2,height=2,frameIndex=42,timestamp=1234000,get_buffer_as_uint16=lambda:buffer)
        capture=AstraDepth.__new__(AstraDepth);capture.sdk=Mock();capture.stream=Mock();capture.serial='astra-test'
        capture.stream.read_frame.return_value=frame
        raw,meta=capture.read();buffer[:]=bytes(len(buffer))
        np.testing.assert_array_equal(raw,[[0,1000],[2000,3000]])
        self.assertEqual(meta['sensor_timestamp_ms'],1234.)
        self.assertEqual(meta['pairing'],'independent_unsynchronized')
        self.assertEqual(meta['registration'],'native_depth_unregistered')

if __name__=="__main__":unittest.main()
