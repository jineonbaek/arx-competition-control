"""Competition first-leg uses straight controller, with inert ROS197 transport."""
import unittest
from pathlib import Path
from test_competition_start_direct_ros import CompetitionStart as Fixture

class EntryAll(unittest.TestCase):
 setUpClass=classmethod(Fixture.setUpClass.__func__);tearDownClass=classmethod(Fixture.tearDownClass.__func__)
 setUp=Fixture.setUp;tearDown=Fixture.tearDown;refresh=Fixture.refresh;add=Fixture.add;worker=Fixture.worker
 def test_near_single_special_ignores_occupied_maps(self):
  self.blocked=True;self.refresh();self.add('special',.05);w=self.n.waypoints;before=w.route.definition();w.start(None)
  self.assertEqual(w.route.mode,'direct');self.assertEqual(w.route.index,0);self.assertTrue(self.n._special_execution)
  self.n.seen=self.n.seq;self.simulate=True;self.assertEqual(self.n.run_goal(),'arrived_stopped')
  self.n.cancel('arrived_stopped');self.assertEqual(w.route.state,'completed');self.assertEqual(w.route.definition(),before)
 def test_near_first_before_t1_keeps_direct_without_map_planning(self):
  self.worker();self.blocked=True;self.refresh();self.add('special',.05);self.add('task',.4,'point_1')
  w=self.n.waypoints;w.start(None);self.assertEqual(w.route.mode,'direct');self.assertEqual(w.route.target_index,0)
  self.assertEqual(w.route.direct_source['pose'],[0.,0.,0.]);self.assertEqual(self.n.transport.calls,[])
 def test_first_via_ignores_missing_maps_and_retains_saved_type(self):
  self.missing=True;self.refresh();self.add('via',.18);w=self.n.waypoints;before=w.route.definition();w.start(None)
  self.assertEqual(w.route.mode,'direct');self.assertEqual(w.route.target_index,0);self.assertTrue(self.n._special_execution)
  self.n.seen=self.n.seq;self.simulate=True;self.assertEqual(self.n.run_goal(),'arrived_stopped');self.n.cancel('arrived_stopped')
  self.assertEqual(w.route.state,'completed');self.assertEqual(w.route.definition(),before)
 def test_first_stop_does_not_pull_following_special_into_entry_leg(self):
  self.add('stop',.18);self.add('special',.4);w=self.n.waypoints;w.start(None)
  self.assertEqual(w.route.mode,'direct');self.assertEqual(w.route.target_index,0);self.assertEqual(len(w.route.direct_points()),1)
if __name__=='__main__':unittest.main()
