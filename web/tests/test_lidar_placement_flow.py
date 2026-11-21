import time
import unittest
from unittest.mock import patch
import numpy as np
import test_navigation_placement as fixtures
from mouth_push import placement_timing,reobserve_seat


class LidarHandoffTests(unittest.TestCase):
    def test_stopped_lidar_visit_starts_arm_without_visual_chassis_command(self):
        fx=fixtures.NavigationPlacementTests();fx.setUp()
        try:
            fx.module.lidar_locked=True
            fx.record(False)
            fx.module._stop_chassis=lambda c,r:dict(stopped=True,pose=dict(x=.2,y=.4,yaw=.1))
            fx.module._observe=lambda *a:self.fail('D435 chassis observation')
            fx.module._chassis_step=lambda *a:self.fail('chassis command')
            fx.module.start(fx.context);result=fx.wait()
            self.assertEqual(result['status'],'completed',result)
            handoff=fx.arm.starts[0][1]
            self.assertEqual(handoff['admission_source'],'lidar_point_2')
            self.assertEqual(handoff['navigation_visit_id'],fx.nav.visit['visit_id'])
            with self.assertRaises(ValueError):fx.module.start(fx.context)
        finally:fx.doCleanups()


class PlacementFlowTests(unittest.TestCase):
    def test_speed_limits_never_exceed_configured_envelope(self):
        caps=dict(tcp_speed_mm_s=60.,tcp_acceleration_mm_s2=120.,joint_speed_deg_s=[15.]*6)
        for base_speed, cruise, tail in [(8.,40.,24.),(30.,60.,24.)]:
            for terminal, speed, accel in [(False,cruise,100.),(True,tail,36.)]:
                with self.subTest(base_speed=base_speed,terminal=terminal):
                    timing=placement_timing(caps,base_speed,terminal)
                    self.assertEqual(timing['tcp_speed_mm_s'],speed)
                    self.assertEqual(timing['tcp_acceleration_mm_s2'],accel)
                    self.assertEqual(timing['joint_speed_deg_s'],caps['joint_speed_deg_s'])
        low=dict(caps,tcp_speed_mm_s=10.,tcp_acceleration_mm_s2=20.)
        tail=placement_timing(low,30.,True)
        self.assertAlmostEqual(tail['tcp_speed_mm_s'],6.)
        self.assertAlmostEqual(tail['tcp_acceleration_mm_s2'],7.2)

    def test_missing_frames_are_observed_again_without_replaying_motion(self):
        rows=iter([{},dict(allow_compensation=False),dict(insertion_depth_verified=True)])
        events=[]
        result=reobserve_seat(lambda:next(rows),lambda:True,lambda *a:events.append(a),lambda s:None)
        self.assertTrue(result['insertion_depth_verified']);self.assertEqual(len(events),2)

    def test_stop_interrupts_observation_wait(self):
        def wait(_):raise ValueError('stopped')
        with self.assertRaisesRegex(ValueError,'stopped'):
            reobserve_seat(lambda:{},lambda:True,lambda *a:None,wait)

class ReversePlacementTests(unittest.TestCase):
    def exercise(self,failure):
        from fast_visual_level import return_after_release
        from docking_motion import ArmStartChanged
        from types import SimpleNamespace
        q=np.zeros(6);q[0]=.06
        state=lambda:dict(arms={'right':{'joints':q.tolist()}},body={'height':0.})
        driver=dict(driver_instance='original',command_epoch=7)
        ctrl=SimpleNamespace(snapshot=state,adapter=SimpleNamespace(trajectory_status=lambda _:dict(driver)))
        model=SimpleNamespace(reach=.5,wrist_transform=lambda *_:np.eye(4))
        starts=[];sent=[]
        def send(parts,guard,expected_start,expected_driver):
            guard();starts.append(expected_start['joints'][0])
            if len(starts)==1:
                q[0]+=.003
                if failure=='replaced':driver['command_epoch']+=1
                if failure=='unknown':raise ValueError('uncertain dispatch')
                raise ArmStartChanged('not dispatched')
            sent.append(parts[0]);q[:]=parts[0]['joint_waypoints_rad'][-1]
        motion=SimpleNamespace(arm=send,settled_arm=lambda guard:state())
        limits=dict(joint_speed_deg_s=[15.]*6,joint_acceleration_deg_s2=[18.]*6,
                    tcp_speed_mm_s=60.,tcp_acceleration_mm_s2=120.)
        invoke=lambda:return_after_release(ctrl,motion,model,np.eye(4),[q.tolist(),[0.]*6],limits,
                                           lambda:True,'original')
        return invoke,starts,sent

    def test_retimes_from_unloaded_feedback_and_sends_once(self):
        invoke,starts,sent=self.exercise('changed')
        invoke()
        np.testing.assert_allclose(starts,[.06,.063])
        self.assertEqual(len(sent),1)
        self.assertAlmostEqual(sent[0]['joint_waypoints_rad'][0][0],.063)

    def test_never_reissues_motion_after_uncertain_dispatch_or_driver_change(self):
        for failure in ('unknown','replaced'):
            invoke,starts,sent=self.exercise(failure)
            with self.assertRaises(ValueError):invoke()
            self.assertEqual(len(starts),1)
            self.assertEqual(sent,[])
