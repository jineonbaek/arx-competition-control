from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import fast_visual_level as fast
from agent.visual_chassis_correction import finish_visual_chassis_workflow


class Controller:
    def __init__(self):self.token=object();self.reservations=0;self.releases=0;self.adapter=self
    def motion_valid(self,token):return token is self.token
    def snapshot(self):return dict(chassis_active=False,command_epoch=1,
        arms={'right':{'gripper':-.7,'joints':[0.]*6}},body={'height':0.})
    def reserve_motion(self,epoch):self.reservations+=1;raise AssertionError('Must reuse competition ownership')
    def release_motion(self,token):self.releases+=1;raise AssertionError('Competition must release its own token')
    def trajectory_status(self,side):raise ValueError('injected driver fault before any motion')


class NavigationOwnedPlacementTests(unittest.TestCase):
    def handoff(self,age=0.):
        return finish_visual_chassis_workflow(admitted=True,reason='test',chassis_pose={},now=time.time()-age)

    def test_accepts_fresh_admission_and_never_reserves_another_token(self):
        ctrl=Controller();task=fast.FastVisualLevel(ctrl,None);event=threading.Event();task._run=event.set
        task.motion=object()
        with tempfile.TemporaryDirectory() as folder,patch.object(fast,'WORK',Path(folder)):
            state=task.start_owned(ctrl.token,self.handoff());task.thread.join(1.)
        self.assertTrue(event.is_set());self.assertEqual(ctrl.reservations,0)
        self.assertIs(task.owned_token,ctrl.token)
        self.assertIsNone(task.motion)
        self.assertEqual(state['mode'],'auto_insert_160')

    def test_stale_admission_cannot_launch_arm(self):
        ctrl=Controller();task=fast.FastVisualLevel(ctrl,None)
        with self.assertRaises(ValueError):task.start_owned(ctrl.token,self.handoff(age=10.))
        self.assertIsNone(task.thread)

    def test_early_driver_fault_does_not_release_competition_token(self):
        ctrl=Controller();task=fast.FastVisualLevel(ctrl,None)
        task.owned_token=ctrl.token;task.auto_insert_160=True
        task._run()
        self.assertEqual(task.snapshot()['status'],'complete_degraded')
        self.assertEqual(ctrl.reservations,0);self.assertEqual(ctrl.releases,0)

if __name__=='__main__':unittest.main()
