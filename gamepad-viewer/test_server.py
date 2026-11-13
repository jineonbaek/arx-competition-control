import struct
import unittest
from server import State, normalize
from profiles import XINPUT

class InputTests(unittest.TestCase):
    def test_measured_centers_and_endpoints(self):
        for index,center,lo,hi in [(0,-16380,-32767,0),(1,16379,0,32767),(2,-16380,-32767,0),(3,16379,0,32767)]:
            self.assertAlmostEqual(normalize(index,center),0)
            self.assertEqual(normalize(index,lo),-1)
            self.assertEqual(normalize(index,hi),1)
        self.assertEqual(normalize(0,32767),1)
    def test_initial_and_live_events(self):
        s=State();s.event(struct.pack('IhBB',0,1,0x81,13))
        self.assertEqual(s.snapshot()['buttons'][13],1)
        s.event(struct.pack('IhBB',1,0,1,13))
        self.assertEqual(s.snapshot()['buttons'][13],0)
        s.event(struct.pack('IhBB',2,-32767,2,4))
        self.assertEqual(s.snapshot()['axes'][4],-32767)
    def test_disconnect_clears_inputs(self):
        s=State();s.event(struct.pack('IhBB',0,1,1,12));s.disconnect()
        self.assertFalse(s.snapshot()['connected'])
        self.assertEqual(s.snapshot()['buttons'],[0]*14)
        self.assertEqual(s.snapshot()['normalized'],[0]*6)
    def test_initialization_and_disconnect(self):
        s=State()
        self.assertFalse(s.snapshot()["initialized"])
        for index in range(6):
            s.event(struct.pack("IhBB",0,0,0x82,index))
        s.event(struct.pack("IhBB",0,0,0x81,7))
        self.assertTrue(s.snapshot()["initialized"])
        s.disconnect()
        self.assertFalse(s.snapshot()["initialized"])

    def test_xinput_initialization_and_logical_axes(self):
        s=State(XINPUT)
        s.connect(XINPUT,"Generic X-Box pad")
        for i,v in enumerate([0,0,-32767,32767,-32767,-32767,0,0]):
            s.event(struct.pack("IhBB",0,v,0x82,i))
        snapshot=s.snapshot()
        self.assertTrue(snapshot["initialized"])
        self.assertEqual(snapshot["logical_axes"],[0,0,1,-1,0,0])
        self.assertFalse(snapshot["lt_pressed"])
        self.assertEqual(snapshot["mode"],"XINPUT")

    def test_mode_change_clears_inputs_and_changes_session(self):
        s=State();s.connect(XINPUT,"Generic X-Box pad")
        before=s.snapshot()["device_session"]
        s.event(struct.pack("IhBB",0,1,1,1))
        from profiles import NS
        s.connect(NS,"BEITONG BTP-KP20 NS")
        after=s.snapshot()
        self.assertNotEqual(before,after["device_session"])
        self.assertFalse(after["initialized"])
        self.assertFalse(after["lt_pressed"])
        self.assertEqual(after["buttons"],[0]*14)

    def test_unknown_mode_reports_error_and_disables_input(self):
        s=State(XINPUT);s.connect(XINPUT,"Generic X-Box pad")
        s.event(struct.pack("IhBB",0,32767,2,2))
        self.assertTrue(s.snapshot()["lt_pressed"])
        s.disconnect("未知手柄模式；禁止底盘输出")
        result=s.snapshot()
        self.assertFalse(result["connected"])
        self.assertFalse(result["lt_pressed"])
        self.assertIsNone(result["mode"])
        self.assertIn("未知手柄模式",result["error"])

    def test_unknown_index_ignored(self):
        s=State();s.event(struct.pack('IhBB',0,1,1,200))
        self.assertEqual(s.snapshot()['buttons'],[0]*14)

if __name__=='__main__': unittest.main()
