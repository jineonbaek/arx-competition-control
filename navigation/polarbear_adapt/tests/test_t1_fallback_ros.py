"""T1-only relaxed third stopped check, real handoff, private/inert ROS."""
import math,time,unittest
import numpy as np,rclpy
import test_mission_ros as f
import test_terminal_correction_ros as correction

class T1Fallback(unittest.TestCase):
    setUpClass=classmethod(lambda cls:rclpy.init())
    tearDownClass=classmethod(lambda cls:rclpy.shutdown())
    setUp=f.MissionROS.setUp;tearDown=f.MissionROS.tearDown
    refresh=f.MissionROS.refresh;add=f.MissionROS.add

    def prepare(self,offset=.0699,kind='task',tag='point_1',yaw=.0646,goal=.2):
        self.h,self.token=f.MissionROS.worker(self)
        self.heartbeat=self.n.create_timer(.2,lambda:self.h.heartbeat(self.token))
        self.position[:]=[goal-offset,0.,yaw];self.refresh()
        self.add(kind,goal,tag if kind=='task' else None)
        self.n.waypoints.start(None);self.n.seen=self.n.seq
        # Exercise the original MPPI fallback, independently of task_direct.
        if kind=='task':self.n._special_execution=False;self.n.waypoints.route.mode='navigation'
        self.n.terminal_correction=True;self.n.terminal_checks=2

    def test_third_t1_check_below_7cm_produces_real_point_1_notification(self):
        self.prepare();result=self.n.verify_arrival()
        self.assertEqual(result,'arrived_stopped')
        self.assertIsNone(self.h.active)
        self.n.cancel(result)
        self.assertEqual(self.h.active['point_id'],'point_1')
        self.assertEqual(self.h.active['status'],'notified')
        self.assertAlmostEqual(abs(self.h.active['arrival_pose'][0]-self.h.active['goal_pose'][0]),.0699)
        self.assertTrue(all(not any(c) for c in self.n.transport.calls))

    def test_real_three_check_loop_notifies_t1_once_after_two_retries(self):
        correction.TerminalCorrection.prepare(self,persistent=True,task_id='point_1')
        with self.n.transport.task_scope(self.identity):result=self.n.run_goal()
        self.assertEqual(result,'arrived_stopped');self.assertEqual(self.client.requests,3)
        self.assertEqual(self.n.terminal_checks,2);self.assertIsNone(self.task_hub.active)
        self.assertEqual(self.n.transport.current,self.identity)
        self.n.cancel(result)
        self.assertEqual(self.task_hub.active['point_id'],'point_1')
        self.assertEqual(self.task_hub.active['status'],'notified')

    def test_first_two_t1_checks_remain_45mm(self):
        self.prepare()
        for count in (0,1):
            self.n.terminal_checks=count
            with self.assertRaisesRegex(ValueError,'outside_arrival_tolerance'):self.n.verify_arrival()
        self.assertIsNone(self.h.active)

    def test_exact_7cm_is_not_accepted(self):
        self.prepare(offset=.07,goal=.07,yaw=0.)
        with self.assertRaisesRegex(ValueError,'outside_arrival_tolerance'):self.n.verify_arrival()
        self.assertIsNone(self.h.active)

    def test_exact_7cm_with_rotated_body_is_not_accepted(self):
        self.prepare(offset=.07,goal=.07,yaw=.0646)
        with self.assertRaisesRegex(ValueError,'outside_arrival_tolerance'):self.n.verify_arrival()
        self.assertIsNone(self.h.active)

    def test_t2_keeps_original_tolerance(self):
        self.prepare(tag='point_2')
        with self.assertRaisesRegex(ValueError,'outside_arrival_tolerance'):self.n.verify_arrival()
        self.assertIsNone(self.h.active)

    def test_plain_goal_cannot_use_t1_fallback(self):
        self.prepare(kind='stop')
        with self.assertRaisesRegex(ValueError,'outside_arrival_tolerance'):self.n.verify_arrival()

    def test_wrong_semantic_target_cannot_borrow_t1_fallback(self):
        self.prepare();self.n.waypoints.route.points[0]['pose'][0]+=.001
        with self.assertRaisesRegex(ValueError,'outside_arrival_tolerance'):self.n.verify_arrival()

    def test_yaw_outside_original_tolerance_still_blocks_notification(self):
        self.prepare(offset=.0533,yaw=.10)
        with self.assertRaisesRegex(ValueError,'outside_arrival_tolerance'):self.n.verify_arrival()
        self.assertIsNone(self.h.active)

    def test_occupied_footprint_cannot_use_position_fallback(self):
        self.prepare()
        for _,g in self.n.grids.values():
            col,row=np.floor((self.position[:2]-g.origin)/g.res).astype(int);g.grid[row,col]=100
        with self.assertRaisesRegex(ValueError,'footprint_blocked'):self.n.verify_arrival()
        self.assertIsNone(self.h.active)

    def test_old_source_cannot_use_position_fallback(self):
        self.prepare();self.n.pose_snapshot.header.stamp.sec-=2
        with self.assertRaises(ValueError):self.n.verify_arrival()
        self.assertIsNone(self.h.active)

    def test_not_stationary_cannot_use_position_fallback(self):
        self.prepare();self.n.destroy_timer(self.timer);began=time.monotonic()
        def feed():
            self.position[0]=.1467+.02*math.sin((time.monotonic()-began)*25);self.refresh()
        self.timer=self.n.create_timer(.015,feed)
        with self.assertRaisesRegex(ValueError,'not_stationary'):self.n.verify_arrival()
        self.assertIsNone(self.h.active)

if __name__=='__main__':unittest.main()
