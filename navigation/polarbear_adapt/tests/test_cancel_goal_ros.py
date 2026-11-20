"""Cancellation/replacement boundaries on the real ROS driver, inert transport.

Run only on isolated ROS domain 197/198 as required by test_mission_ros.
"""
import copy
import math
import unittest
from geometry_msgs.msg import PoseStamped
import test_mission_ros as fixture

class CancelGoalROS(unittest.TestCase):
    setUpClass=classmethod(lambda cls:fixture.rclpy.init())
    tearDownClass=classmethod(lambda cls:fixture.rclpy.shutdown())
    setUp=fixture.MissionROS.setUp
    tearDown=fixture.MissionROS.tearDown
    refresh=fixture.MissionROS.refresh
    add=fixture.MissionROS.add

    def goal(self,x=2.):
        m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.n.get_clock().now().to_msg()
        m.pose.position.x=x;m.pose.orientation.w=1.;return m

    def reject_ends(self):
        self.reject=True;self.payloads=[];submit=self.n.transport.submit
        def guarded(payload):
            self.payloads.append(dict(payload))
            if self.reject and payload['type']=='chassis.end_output':return dict(ok=False)
            return submit(payload)
        self.n.transport.submit=guarded

    def test_cancel_retires_epoch_and_timer_retries_scoped_end(self):
        n=self.n;self.add('stop',1.);n.waypoints.start(None);n.seen=n.seq
        old=n.transport.current;epoch=n.seq;self.reject_ends()
        n.cancel('cancelled')
        self.assertGreater(n.seq,epoch);self.assertIsNone(n.target);self.assertFalse(n.waypoints.route.running)
        self.assertIn(old,n.pending_output_ends)
        with self.assertRaisesRegex(ValueError,'goal_changed'):n.current_intent(epoch)
        self.reject=False;n.spin(.25)
        self.assertFalse(n.pending_output_ends);self.assertIn(old,n.transport.ended)
        self.assertIsNone(n.target)
        self.assertTrue(all(p['type']=='chassis.end_output' for p in self.payloads))
        self.assertEqual({(p['source_id'],p['task_id'],p['generation']) for p in self.payloads},
                         {(old.source_id,old.task_id,old.generation)})

    def test_new_valid_goal_preserved_while_output_generation_waits_for_old_end(self):
        n=self.n;self.add('stop',1.);n.waypoints.start(None);self.reject_ends()
        n.cancel('cancelled');old=n.transport.current;n.accept_goal(self.goal())
        fixture.np.testing.assert_allclose(n.target,[2.,0.,0.]);self.assertEqual(n.transport.current,old)
        n.seen=n.seq
        with self.assertRaisesRegex(ValueError,'previous_output_end_unconfirmed'):n.current_intent(n.seq)
        self.assertTrue(all(p['type']=='chassis.end_output' for p in self.payloads))
        self.reject=False;n.spin(.25)
        self.assertNotEqual(n.transport.current,old);fixture.np.testing.assert_allclose(n.target,[2.,0.,0.])
        self.assertTrue(all(p['type']=='chassis.end_output' for p in self.payloads))

    def test_invalid_and_delayed_arrows_preserve_running_route_progress(self):
        n=self.n;self.add('via',.3);self.add('stop',1.);n.waypoints.start(None)
        old=n.transport.current;epoch=n.seq;target=n.target.copy();definition=copy.deepcopy(n.waypoints.route.definition())
        index=n.waypoints.route.index;calls=len(n.transport.calls)
        for kind in ('old','frame','nan','quaternion'):
            with self.subTest(kind=kind):
                m=self.goal()
                if kind=='old':m.header.stamp.sec-=10
                elif kind=='frame':m.header.frame_id='odom'
                elif kind=='nan':m.pose.position.x=math.nan
                else:m.pose.orientation.w=0.
                n.plan_changed(m)
                fixture.np.testing.assert_allclose(n.target,target);self.assertEqual(n.seq,epoch)
                self.assertEqual(n.transport.current,old);self.assertEqual(n.waypoints.route.definition(),definition)
                self.assertEqual(n.waypoints.route.index,index);self.assertTrue(n.waypoints.route.running)
                self.assertEqual(len(n.transport.calls),calls)

    def test_arrival_advances_once_only_after_old_output_end_acknowledged(self):
        n=self.n;self.add('stop',.2);self.add('stop',.8);n.waypoints.start(None)
        self.position[0]=.2;self.refresh();self.reject_ends()
        n.cancel('arrived_stopped')
        self.assertEqual(n.waypoints.route.index,0);self.assertIsNone(n.target)
        self.reject=False;n.spin(.25)
        self.assertEqual(n.waypoints.route.index,1);fixture.np.testing.assert_allclose(n.target,[.8,0.,0.])
        n.spin(.15);self.assertEqual(n.waypoints.route.index,1)

if __name__=='__main__':unittest.main()
