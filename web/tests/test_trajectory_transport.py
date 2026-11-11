"""Fake ROS messages only: no node, SDK or hardware is constructed."""
import copy
import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from test_control import configured, Rejected
from ros_adapter import RosAdapter


class TrajectoryTransportTests(unittest.TestCase):
    def setUp(self):
        self.adapter=RosAdapter(configured())
        self.adapter.node=object()
        self.adapter.String=SimpleNamespace
        self.sent=[]
        self.ok=True
        def publish(msg):
            p=json.loads(msg.data);self.sent.append(p)
            if p['kind']=='start':
                event,result=self.adapter.pending_arm_commands[p['request_id']]
                result.update(ok=self.ok,message='rejected fixture' if not self.ok else 'accepted')
                event.set()
        self.adapter._publisher=lambda *args:SimpleNamespace(publish=publish)
        self.state=dict(driver_instance='driver-one',command_epoch=7,enabled=True,request_id='',
                        status='idle',reason='',ready=True,manual_joint_active=False)
        self.plan=dict(interpolation='segment_quintic_scalar',time_from_start_s=[0.,2.],
                       joint_waypoints_rad=[[0.]*6,[.01]*6])
        self.adapter.update_trajectory('right',self.state)

    def test_single_full_plan_and_confirmation_do_not_mean_arrival(self):
        handle,confirm=self.adapter.trajectory_start('right',self.plan,self.state)
        self.assertEqual(len(self.sent),1)
        self.assertEqual(self.sent[0]['joint_waypoints_rad'],self.plan['joint_waypoints_rad'])
        self.assertEqual(self.sent[0]['command_epoch'],7)
        self.assertEqual(self.sent[0]['driver_instance'],'driver-one')
        self.assertIsNone(confirm())
        self.assertEqual(self.adapter.pending_arm_commands,{})
        self.assertNotIn('arrived',handle)
        with self.assertRaises(Rejected):self.adapter.trajectory_keepalive('right',handle)
        self.assertEqual(len(self.sent),1)
        self.adapter.update_trajectory('right',{**self.state,'status':'running','command_epoch':8,'request_id':handle['request_id']})
        self.adapter.trajectory_keepalive('right',handle)
        self.assertEqual(self.sent[-1]['kind'],'keepalive')
        self.assertNotIn('joint_waypoints_rad',self.sent[-1])

    def test_stale_disabled_and_changed_driver_reject_without_publish(self):
        for changes in ({'enabled':False},{'ready':False},{'manual_joint_active':True},
                        {'driver_instance':'new-driver'},{'command_epoch':8},{'status':'running'}):
            self.adapter.update_trajectory('right',{**self.state,**changes})
            with self.assertRaises(Rejected):self.adapter.trajectory_start('right',self.plan,self.state)
        self.adapter.update_trajectory('right',self.state)
        with patch('ros_adapter.time.monotonic',return_value=1e30):
            self.assertIsNone(self.adapter.trajectory_status('right'))
            with self.assertRaises(Rejected):self.adapter.trajectory_start('right',self.plan,self.state)
        self.assertEqual(self.sent,[])

    def test_invalid_input_and_policy_never_publish(self):
        for field,value in [('interpolation','linear'),('time_from_start_s',[0,0]),
                            ('joint_waypoints_rad',[[0.]*6,[float('nan')]*6]),
                            ('joint_waypoints_rad',[[0.]*6,[True]*6]),
                            ('joint_waypoints_rad',[[0.]*6,[10.]*6])]:
            plan=copy.deepcopy(self.plan);plan[field]=value
            with self.assertRaises((ValueError,Rejected)):self.adapter.trajectory_start('right',plan,self.state)
        for limits in ([], [[0.,1.]]*5, [[0.,float('inf')]]*6):
            self.adapter.config['arms']['right']['joint_limits_deg']=limits
            with self.assertRaises(Rejected):self.adapter.trajectory_start('right',self.plan,self.state)
        self.assertEqual(self.sent,[])
        self.adapter.update_trajectory('right',{**self.state,'command_epoch':True})
        self.assertIsNone(self.adapter.trajectory_status('right'))

    def test_driver_rejection_publish_error_and_timeout_clear_pending(self):
        self.ok=False
        _,confirm=self.adapter.trajectory_start('right',self.plan,self.state)
        with self.assertRaisesRegex(Rejected,'rejected fixture'):confirm()
        self.assertEqual(self.adapter.pending_arm_commands,{})
        self.adapter._publisher=lambda *args:SimpleNamespace(publish=lambda msg:None)
        _,confirm=self.adapter.trajectory_start('right',self.plan,self.state)
        with patch('threading.Event.wait',return_value=False):
            with self.assertRaisesRegex(Rejected,'不确定'):confirm()
        self.assertEqual(self.adapter.pending_arm_commands,{})
        def fail(msg):raise RuntimeError('publish failed')
        self.adapter._publisher=lambda *args:SimpleNamespace(publish=fail)
        with self.assertRaises(RuntimeError):self.adapter.trajectory_start('right',self.plan,self.state)
        self.assertEqual(self.adapter.pending_arm_commands,{})

    def test_terminal_wrong_id_and_new_driver_cannot_receive_old_keepalive(self):
        handle,confirm=self.adapter.trajectory_start('right',self.plan,self.state);confirm()
        running={**self.state,'status':'running','command_epoch':8,'request_id':handle['request_id']}
        for changes in ({'status':'failed'},{'status':'reference_complete'},
                        {'request_id':'other'},{'driver_instance':'restarted'},{'command_epoch':9}):
            self.adapter.update_trajectory('right',{**running,**changes})
            with self.assertRaises(Rejected):self.adapter.trajectory_keepalive('right',handle)
        self.assertEqual(len(self.sent),1)

if __name__=='__main__':unittest.main()
