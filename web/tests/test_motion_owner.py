"""Controller ownership using FakeAdapter only; no device access."""
import unittest
from test_control import Controller,FakeAdapter,configured,Rejected

class MotionOwnerTests(unittest.TestCase):
    def setUp(self):
        self.now=100.
        self.adapter=FakeAdapter()
        self.controller=Controller(configured(),self.adapter,lambda:self.now)
        for side in ('left','right'):
            self.controller.update_arm(side,[0.]*6,-1.5,[0.]*6)
        self.controller.update_body(5.,[0.]*4,0.,0.,0.)

    def reserve(self):
        return self.controller.reserve_motion(self.controller.command_epoch)

    def test_reservation_has_no_hardware_side_effect_and_token_not_exposed(self):
        token=self.reserve()
        self.assertEqual(self.adapter.calls,[])
        self.assertTrue(self.controller.motion_valid(token))
        state=self.controller.snapshot()
        self.assertTrue(state['motion_reserved'])
        self.assertFalse(state['motion_interrupted'])
        self.assertNotIn(token,state.values())
        self.assertEqual(state['command_epoch'],1)

    def test_all_other_motion_is_blocked_but_owner_uses_existing_validation(self):
        token=self.reserve()
        commands=[{'type':'arm.joints','arm':'right','joints':[1.]*6},
            {'type':'arm.joints','arm':'left','joints':[1.]*6},
            {'type':'arm.mode','arm':'right','mode':1},
            {'type':'gripper','arm':'right','position':-1.2},
            {'type':'arm.tcp','arm':'right','position_mm':[0.]*3,'orientation_deg':[0.]*3},
            {'type':'arm.wrist','arm':'right','frame':'base_link','position_mm':[250,-250,500],
             'orientation_deg':[0,0,0]},
            {'type':'lift','target':6.},
            {'type':'chassis','x':.01,'y':0.,'omega':0.,'sequence':1},
            {'type':'chassis.nav_sdk','x':.01,'y':0.,'omega':0.,'sequence':1}]
        for command in commands:
            with self.subTest(command=command),self.assertRaisesRegex(Rejected,'控制权'):
                self.controller.submit(command,'browser')
        self.assertEqual(self.adapter.calls,[])
        self.controller.submit(commands[0],token)
        self.assertEqual(len(self.adapter.calls),1)
        with self.assertRaises(Rejected):
            self.controller.submit({'type':'arm.joints','arm':'right','joints':[999.]*6},token)
        self.assertEqual(len(self.adapter.calls),1)

    def test_stops_interrupt_old_task_and_cannot_be_undone_by_owner(self):
        for stop in ({'type':'arm.stop','arm':'right'},{'type':'lift.stop'},
                     {'type':'chassis.stop','sequence':1}):
            with self.subTest(stop=stop):
                self.setUp();token=self.reserve()
                self.controller.submit(stop,'browser')
                self.assertEqual(len(self.adapter.calls),1)
                self.assertFalse(self.controller.motion_valid(token))
                self.assertTrue(self.controller.snapshot()['motion_interrupted'])
                with self.assertRaises(Rejected):
                    self.controller.submit({'type':'lift','target':6.},token)
                self.controller.submit({'type':'arm.stop','arm':'right'},token)
                self.assertEqual(len(self.adapter.calls),2)

    def test_owner_stop_for_checkpoint_does_not_interrupt_itself(self):
        token=self.reserve()
        self.controller.submit({'type':'arm.stop','arm':'right'},token)
        self.assertTrue(self.controller.motion_valid(token))

    def test_release_and_new_reservation_invalidate_old_token(self):
        old=self.reserve()
        with self.assertRaises(Rejected):self.controller.release_motion(object())
        self.controller.release_motion(old)
        self.assertFalse(self.controller.motion_valid(old))
        with self.assertRaises(Rejected):
            self.controller.submit({'type':'lift','target':6.},old)
        new=self.reserve();self.assertTrue(self.controller.motion_valid(new))
        with self.assertRaises(Rejected):
            self.controller.submit({'type':'gripper','arm':'right','position':-1.2},old)
        self.assertEqual(self.adapter.calls,[])

    def test_double_reservation_stale_epoch_and_disabled_control_reject(self):
        token=self.reserve()
        with self.assertRaises(Rejected):self.reserve()
        self.controller.release_motion(token)
        for epoch in (0,True,None):
            with self.assertRaises(Rejected):self.controller.reserve_motion(epoch)
        self.controller.config['execution_enabled']=False
        with self.assertRaises(Rejected):self.reserve()
        self.assertEqual(self.adapter.calls,[])

    def test_active_chassis_cannot_be_taken_over(self):
        self.controller.submit({'type':'chassis','x':.01,'y':0.,'omega':0.,'sequence':1},'browser')
        with self.assertRaises(Rejected):self.reserve()
        self.assertEqual(len(self.adapter.calls),1)

    def test_other_browser_stop_preempts_owned_chassis(self):
        token=self.reserve()
        self.controller.submit({'type':'chassis','x':.01,'y':0.,'omega':0.,'sequence':1},token)
        self.controller.submit({'type':'chassis.stop','sequence':1},'browser')
        self.assertFalse(self.controller.motion_valid(token))
        self.assertIsNone(self.controller.chassis_owner)
        self.assertEqual(self.adapter.calls[-1][1][:3],(0,0,0))

    def test_reservation_during_unlocked_wrist_ik_invalidates_old_request(self):
        from unittest.mock import patch
        c=self.controller
        def inverse(*args):
            self.reserve()
            return [0.]*6
        with patch.object(c,'wrist_reason',return_value=''), \
             patch.object(c.models['right'],'inverse_wrist',side_effect=inverse):
            with self.assertRaisesRegex(Rejected,'控制权'):
                c.submit({'type':'arm.wrist','arm':'right','frame':'base_link',
                    'position_mm':[250,-250,500],'orientation_deg':[0,0,0]},'browser')
        self.assertEqual(self.adapter.calls,[])

    def test_stopping_failure_still_invalidates_task(self):
        from unittest.mock import patch
        token=self.reserve()
        with patch.object(self.adapter,'lift',side_effect=RuntimeError('link unavailable')):
            with self.assertRaises(RuntimeError):self.controller.submit({'type':'lift.stop'},'browser')
        self.assertFalse(self.controller.motion_valid(token))
        with self.assertRaises(Rejected):self.controller.submit({'type':'lift','target':6.},token)

if __name__=='__main__':unittest.main()
