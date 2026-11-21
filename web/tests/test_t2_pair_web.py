import copy,unittest
from unittest.mock import patch
from test_t2_session import T2SessionTests
from t2_test_task import T2TestTask
from competition_task import CompetitionTask

class PairWeb(unittest.TestCase):
 setUp=T2SessionTests.setUp
 def test_two_point_route_only_registers_real_t2(self):
  self.route.r.points=self.route.r.points[-2:];self.route.show();before=self.route.r.definition()
  self.n.prepare(self.context);self.assertEqual(self.h.state()['point_ids'],['point_2'])
  self.assertEqual(self.n.points,{'point_2':'6'});self.assertEqual(self.route.r.definition(),before);self.assertEqual(self.route.sent,[])
 def test_middle_t2_with_navigation_suffix_is_accepted(self):
  r=self.route.r;r.points=r.points[-2:]+[dict(id='after',type='special',pose=[8.,0.,0.])];self.route.show()
  self.n.begin(self.context);self.assertEqual(self.route.sent,['start_t2']);self.assertEqual(r.execution_end,3)
 def test_duplicate_t2_refused_before_registration_or_publish(self):
  r=self.route.r;r.points.append(dict(r.points[-1],id='duplicate'));self.route.show()
  with self.assertRaises(ValueError):self.n.prepare(self.context)
  self.assertFalse(self.n.active);self.assertEqual(self.route.sent,[]);self.assertFalse(self.h.state()['worker_ready'])
 def test_full_competition_still_refuses_two_point_route(self):
  self.route.r.points=self.route.r.points[-2:];self.route.show()
  with self.assertRaises(ValueError):self.n.validate_start_route('competition')
 def test_label_uses_actual_pair_instead_of_last_two_points(self):
  ps=[dict(id='entry',type='special'),dict(id='t2',type='task',point_id='point_2'),dict(id='after',type='special')]
  value=dict(navigation=dict(route=dict(definition=dict(points=ps))),status='idle')
  obj=object.__new__(T2TestTask);obj.modules={}
  with patch.object(CompetitionTask,'snapshot',return_value=value):snapshot=obj.snapshot()
  self.assertIn('第1点 → 第2点',snapshot['target'])
 def test_unrelated_test_marker_status_prefix_does_not_block_real_pair(self):
  r=self.route.r;r.points=[dict(id='before',type='special',test_task_id='point_1',pose=[-8.,0.,0.])]+r.points[-2:];self.route.show()
  original=self.route.snapshot
  self.route.snapshot=lambda:dict(route=original()['route']|dict(status='测试模式（无真实作业）；待开始 0/3'))
  self.n.prepare(self.context);self.assertEqual(self.h.state()['point_ids'],['point_2'])
