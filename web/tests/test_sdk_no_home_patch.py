"""File-only verification of the fingerprinted vendor startup patch."""
import importlib.util,os,struct,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('startup_patch',ROOT/'scripts/prepare_no_home_sdk.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class SDKStartupTests(unittest.TestCase):
    def setUp(self):
        p=Path(os.environ.get('ARX_SDK_TEST_FILE','/home/arx/arx-competition-control/repos/ARX_X5/ROS2/X5_ws/src/arx_x5_ros2/arx_x5_controller/lib/arx_x5_src/libarx_x5_src.so'))
        self.original=p.read_bytes()

    def test_skips_home_and_clamp_zero_but_retains_protection_and_cleanup(self):
        patched=module.patch_bytes(self.original)
        self.assertEqual(len(patched),len(self.original))
        first=module.file_offset(self.original,0x150f18)
        self.assertEqual(patched[first:first+10],bytes.fromhex('c78534ffffff02000000'))
        jump=module.file_offset(self.original,0x150fcf)
        self.assertEqual(patched[jump],0xe9)
        destination=0x150fcf+5+struct.unpack_from('<i',patched,jump+1)[0]
        self.assertEqual(destination,0x151360)
        # Only these two instructions may differ; callbacks, error clearing,
        # motor enable, feedback/protection loop and cleanup remain byte exact.
        changed={i for i,(a,b) in enumerate(zip(self.original,patched)) if a!=b}
        allowed=set(range(first,first+10))|set(range(jump,jump+7))
        self.assertTrue(changed<=allowed);self.assertEqual(len(changed),8)

    def test_unknown_or_already_patched_binary_is_rejected(self):
        data=bytearray(self.original);data[-1]^=1
        for original in (bytes(data),module.patch_bytes(self.original),b'not an ELF'):
            with self.assertRaises(ValueError):module.patch_bytes(original)

if __name__=='__main__':unittest.main()
