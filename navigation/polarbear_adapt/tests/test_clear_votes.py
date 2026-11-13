import unittest
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from clear_votes import FrameVotes, TransientEvidence


class ClearanceTest(unittest.TestCase):
    def test_window_and_duplicates(self):
        for flags, consecutive, expected in [
            ([1,1,1],True,True), ([1,1,1],False,False),
            ([1,1,0,1,1],False,True), ([1,0,1,0,1],False,False),
        ]:
            votes=FrameVotes()
            for i,value in enumerate(flags):
                result=votes.add(i*100_000_000,[1] if value else [],consecutive)
                self.assertFalse(votes.add(i*100_000_000,[1],consecutive))
            self.assertEqual(1 in result,expected)
        self.assertFalse(votes.add(2_000_000_000,[1]))

    def test_effective_frame_votes(self):
        from clear_votes import EffectiveVotes
        def trial(events):
            window=EffectiveVotes()
            return [bool(window.add(t*1_000_000,{1} if kind=='C' else set(),
                                    {1} if kind=='H' else set())) for t,kind in events]
        self.assertEqual(trial([(0,'C'),(100,'?'),(200,'C'),(300,'C'),(980,'C')]),
                         [False,False,False,False,True])
        self.assertTrue(trial([(0,'C'),(300,'C'),(600,'C'),(3000,'C')])[-1])
        self.assertFalse(trial([(0,'C'),(300,'C'),(600,'H'),(3000,'C'),(3200,'C')])[-1])
        v=EffectiveVotes()
        for t in range(3): v.add(t,{1},())
        v.forget({1})
        self.assertFalse(v.add(4,{1},()))
        self.assertFalse(trial([(0,'C'),(100,'C'),(200,'H'),(300,'C')])[-1])
        self.assertTrue(trial([(0,'C'),(100,'C'),(200,'H'),(300,'C'),(400,'C')])[-1])
        self.assertFalse(trial([(0,'C'),(100,'C'),(100,'C'),(200,'C')])[-1])

    def test_independent_sources_and_changed_height(self):
        world=TransientEvidence()
        origin=np.array([0.,.025,.211]);empty=np.empty((0,3))
        ray=np.array([[2.,.025,.211]])
        old=np.array([[1.025,.025,.211]])
        world.update('head',0,origin,empty,old)
        for s,t in [('bottom',1),('head',2),('bottom',3)]:
            world.update(s,t*100_000_000,origin,ray,empty)
        self.assertTrue(world.cells)
        world.update('bottom',400_000_000,origin,ray,empty)
        self.assertTrue(world.cells)
        world.update('bottom',450_000_000,origin,ray,empty)
        self.assertFalse(world.cells)
        world.update('head',500_000_000,origin,empty,old+[0,0,.55])
        for i in range(6,12):
            world.update('bottom',i*100_000_000,origin,ray,empty)
        self.assertEqual(len(world.cells),1)
        self.assertGreater(next(iter(world.cells.values()))[0],.7)


if __name__=='__main__':
    unittest.main()
