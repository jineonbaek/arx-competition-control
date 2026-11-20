"""Load-reference continuity, with no ROS, SDK or actuator construction."""
import unittest
import numpy as np
from agent.held_placement_geometry import loaded_trajectory_plan
from docking_motion import DockingMotion

class LoadedTrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.actual=np.array([.3,.5,.5,.07,.34,-1.55])
        self.held=self.actual+np.radians([.1,.7,-.6,.1,.2,0.])
        self.plan=dict(interpolation='segment_quintic_scalar',time_from_start_s=[0.,2.,4.],
            joint_waypoints_rad=[self.actual.tolist(),(self.actual+.01).tolist(),(self.actual+.02).tolist()])
        self.driver={'joint_reference_rad':self.held.tolist()}
        self.limits=[[-180.,180.]]*6

    def test_first_sample_preserves_reference_and_whole_curve_keeps_joint_deltas(self):
        new,offset=loaded_trajectory_plan(self.plan,self.actual,self.driver,self.limits)
        q=np.asarray(new['joint_waypoints_rad'])
        np.testing.assert_array_equal(q[0],self.held)
        np.testing.assert_allclose(np.diff(q,axis=0),np.diff(self.plan['joint_waypoints_rad'],axis=0),atol=1e-15)
        self.assertEqual(new['time_from_start_s'],[0.,2.,4.])
        self.assertEqual(self.plan['joint_waypoints_rad'][0],self.actual.tolist())
        np.testing.assert_allclose(offset,self.held-self.actual)

    def test_already_compensated_curve_is_not_compensated_twice(self):
        p=dict(self.plan,joint_waypoints_rad=[self.held.tolist(),(self.held+.01).tolist()],time_from_start_s=[0.,2.])
        new,offset=loaded_trajectory_plan(p,self.actual,self.driver,self.limits)
        self.assertEqual(new,p);np.testing.assert_array_equal(offset,np.zeros(6))

    def test_bad_or_unbounded_reference_and_out_of_limits_do_not_dispatch(self):
        for held in ([float('nan')]*6,(self.actual+.1).tolist(),[0.]*5):
            with self.assertRaises(ValueError):loaded_trajectory_plan(self.plan,self.actual,{'joint_reference_rad':held},self.limits)
        limits=[[float(np.degrees(a-.01)),float(np.degrees(a+.01))] for a in self.actual]
        with self.assertRaises(ValueError):loaded_trajectory_plan(self.plan,self.actual,self.driver,limits)

    def test_executor_retains_reference_across_two_segments(self):
        test=self
        class Controller:
            def __init__(self):
                self.now=0.;self.q=test.actual.copy();self.ref=test.held.copy();self.epoch=4
                self.adapter=self;self.models={'right':self};self.sent=[]
                self.config={'arms':{'right':{'joint_limits_deg':test.limits}}}
            def clock(self):return self.now
            def sleep(self,duration):self.now+=duration
            def motion_valid(self,token):return token=='owner'
            def snapshot(self):return {'arms':{'right':{'joints':self.q.tolist(),'fresh':True,'received':self.now}},'body':{'height':.4}}
            def trajectory_status(self,side):return {'joint_reference_rad':self.ref.tolist(),'driver_instance':'test','command_epoch':self.epoch}
            def check(self,a,b,h):return {'clear':True}
            def start_trajectory(self,side,plan,token,*,preflight,expected_driver,require_lift_settle):
                test.assertTrue(preflight(self.snapshot(),self.now))
                test.assertEqual(plan['joint_waypoints_rad'][0],self.ref.tolist())
                self.sent.append(plan);old_epoch=self.epoch;self.epoch+=1
                self.ref=np.asarray(plan['joint_waypoints_rad'][-1])
                self.q=self.ref-(test.held-test.actual)
                return {'driver_instance':'test','command_epoch':old_epoch}
            def poll_trajectory(self,token,*,preflight):return {'status':'reference_complete'}
        ctrl=Controller();motion=DockingMotion(ctrl,'owner',guard=lambda s,t:True,sleep=ctrl.sleep,require_lift_settle=False)
        second=dict(self.plan,joint_waypoints_rad=[(self.actual+.02).tolist(),(self.actual+.03).tolist()],time_from_start_s=[0.,2.])
        motion.arm([self.plan,second],lambda:None,expected_driver={'driver_instance':'test','command_epoch':4})
        self.assertEqual(len(ctrl.sent),2);self.assertEqual(len(motion.handoffs),2)
        np.testing.assert_allclose(ctrl.q,self.actual+.03,atol=1e-15)
        self.assertIsNone(motion.active)

if __name__=='__main__':unittest.main()
