import unittest
from teleop import command_from_axes, EnableGate

class TeleopTests(unittest.TestCase):
    def test_cardinal_directions_and_spin(self):
        self.assertEqual(command_from_axes([0,0,0,-1,0,0]), (2,0,0))
        self.assertEqual(command_from_axes([0,0,0,1,0,0]), (-2,0,0))
        self.assertEqual(command_from_axes([0,0,-1,0,0,0]), (0,2,0))
        self.assertEqual(command_from_axes([0,0,1,0,0,0]), (0,-2,0))
        self.assertEqual(command_from_axes([-1,0,0,0,0,0]), (0,0,2))
        self.assertEqual(command_from_axes([1,0,0,0,0,0]), (0,0,-2))
    def test_diagonal_and_deadzone(self):
        x,y,w=command_from_axes([1,0,1,-1,0,0])
        self.assertAlmostEqual((x*x+y*y)**.5,2)
        self.assertEqual(w,-2)
        self.assertEqual(command_from_axes([.04,0,-.04,.04,0,0]),(0,0,0))
    def test_neutral_and_release_required_after_disconnect(self):
        g=EnableGate()
        self.assertFalse(g.update(True, True, True, True))
        self.assertFalse(g.update(True, False, True, True))
        self.assertTrue(g.update(True, True, True, True))
        self.assertFalse(g.update(False, True, True, True))
        self.assertFalse(g.update(True, True, True, True))
        self.assertFalse(g.update(True, False, True, True))
        self.assertFalse(g.update(True, True, False, True))
        self.assertFalse(g.update(True, True, True, True))
        self.assertFalse(g.update(True, False, True, True))
        self.assertTrue(g.update(True, True, True, True))
    def test_fault_requires_rearm(self):
        g=EnableGate();g.update(True,False,True,True)
        self.assertTrue(g.update(True,True,True,True))
        self.assertFalse(g.update(True,True,True,False))
        self.assertFalse(g.update(True,True,True,True))

if __name__=='__main__': unittest.main()
