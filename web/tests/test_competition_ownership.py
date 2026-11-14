import unittest
from test_control import Controller, FakeAdapter, configured, Rejected


class CompetitionOwnershipTests(unittest.TestCase):
    def setUp(self):
        self.adapter=FakeAdapter()
        self.adapter.nav_chassis_sdk=lambda *v:self.adapter.calls.append(('navigation',v))
        self.ctrl=Controller(configured(),self.adapter)
        self.ctrl.update_body(5.,[0.]*4,0.,0.,0.)
        for side in ('left','right'):self.ctrl.update_arm(side,[0.]*6,-1.,[0.]*6)

    def test_manual_blocked_between_navigation_and_grasp_without_motion_reservation(self):
        lease=self.ctrl.reserve_competition(self.ctrl.command_epoch)
        self.assertFalse(self.ctrl.snapshot()['motion_reserved'])
        self.assertTrue(self.ctrl.snapshot()['competition_reserved'])
        for command in [dict(type='lift',target=5.),dict(type='gripper',arm='right',position=-1.),
                        dict(type='chassis',x=.02,y=0.,omega=0.,sequence=1)]:
            with self.assertRaises(Rejected):self.ctrl.submit(command,'manual')
        with self.assertRaises(Rejected):self.ctrl.reserve_motion(self.ctrl.command_epoch)
        self.ctrl.set_competition_navigation(lease,True)
        self.ctrl.submit_navigation(dict(type='chassis.nav_sdk',x=.02,y=0.,omega=0.,sequence=1),'nav')
        self.assertTrue(self.ctrl.snapshot()['chassis_active'])
        self.ctrl.submit_navigation(dict(type='chassis.stop',sequence=2),'nav')
        self.ctrl.set_competition_navigation(lease,False)
        with self.assertRaises(Rejected):
            self.ctrl.submit_navigation(dict(type='chassis.nav_sdk',x=.02,y=0.,omega=0.,sequence=3),'nav')
        token=self.ctrl.reserve_motion(self.ctrl.command_epoch,competition=lease)
        self.ctrl.submit(dict(type='gripper',arm='right',position=-1.),token)
        self.ctrl.release_motion(token)
        with self.assertRaises(Rejected):self.ctrl.submit(dict(type='lift',target=5.),'manual')
        old_navigation=self.ctrl.navigation_client(dict(type='chassis.stop'),'nav')
        self.ctrl.release_competition(lease)
        with self.assertRaises(Rejected):
            self.ctrl.submit(dict(type='chassis.nav_sdk',x=.02,y=0.,omega=0.,sequence=4),old_navigation)
        self.ctrl.submit(dict(type='lift',target=5.),'manual')

    def test_external_stop_interrupts_whole_competition_and_cannot_rearm_navigation(self):
        lease=self.ctrl.reserve_competition(0)
        self.ctrl.set_competition_navigation(lease,True)
        self.ctrl.submit_navigation(dict(type='chassis.nav_sdk',x=.02,y=0.,omega=0.,sequence=1),'nav')
        with self.assertRaises(Rejected):
            self.ctrl.submit_navigation(dict(type='chassis.nav_sdk',x=.02,y=0.,omega=0.,sequence=1),'other-nav')
        self.ctrl.submit(dict(type='chassis.stop',sequence=1),'manual')
        self.assertFalse(self.ctrl.competition_valid(lease))
        with self.assertRaises(Rejected):self.ctrl.set_competition_navigation(lease,True)
        self.ctrl.release_competition(lease)

    def test_standalone_navigation_unchanged_and_release_requires_clean_actuators(self):
        self.ctrl.submit_navigation(dict(type='chassis.nav_sdk',x=.02,y=0.,omega=0.,sequence=1),'nav')
        self.ctrl.submit_navigation(dict(type='chassis.stop',sequence=2),'nav')
        lease=self.ctrl.reserve_competition(self.ctrl.command_epoch)
        token=self.ctrl.reserve_motion(self.ctrl.command_epoch,competition=lease)
        with self.assertRaises(Rejected):self.ctrl.release_competition(lease)
        self.ctrl.release_motion(token);self.ctrl.release_competition(lease)
        with self.assertRaises(Rejected):self.ctrl.set_competition_navigation(lease,True)
