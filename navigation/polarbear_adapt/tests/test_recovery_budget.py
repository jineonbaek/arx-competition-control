"""No SDK/ROS: waiting must not spend the commissioned execution budget."""
import unittest
from unittest.mock import patch
from recovery_budget import paused_budget,charged_motion,require_execution_budget

class BudgetTests(unittest.TestCase):
 def setUp(self):self.clock=100.;self.started=100.;self.seq=1
 @paused_budget
 def wait(self,seconds,inner=0,fail=False):
  self.clock+=seconds
  if inner:self.wait(inner)
  if fail:raise TimeoutError('still waiting')
  return 'ready'
 def test_wait_longer_than_motion_limit_can_still_execute_after_recovery(self):
  with patch('recovery_budget.time.monotonic',lambda:self.clock):
   self.assertEqual(self.wait(500),'ready')
  self.clock+=4
  self.assertEqual(self.clock-self.started,4.)
 def test_nested_waits_are_excluded_once(self):
  with patch('recovery_budget.time.monotonic',lambda:self.clock):self.wait(30,inner=70)
  self.clock+=5
  self.assertEqual(self.clock-self.started,5.)
 def test_repeated_fault_during_wait_does_not_consume_motion_time(self):
  with patch('recovery_budget.time.monotonic',lambda:self.clock):
   with self.assertRaises(TimeoutError):self.wait(100,inner=200,fail=True)
   self.wait(50)
  self.clock+=2
  self.assertEqual(self.clock-self.started,2.)
 def test_active_motion_time_remains_bounded(self):
  self.clock+=481
  with patch('recovery_budget.time.monotonic',lambda:self.clock):self.wait(100)
  self.assertGreater(self.clock-self.started,480.)
 @paused_budget
 def replan_with_escape(self):
  self.clock+=500
  with charged_motion(self):self.clock+=6
 def test_escape_inside_wait_still_consumes_motion_budget(self):
  with patch('recovery_budget.time.monotonic',lambda:self.clock):self.replan_with_escape()
  self.assertEqual(self.clock-self.started,6.)
 @paused_budget
 def replaced_goal(self,escape=False):
  if escape:
   with charged_motion(self):self.replace()
  else:self.replace()
 def replace(self):
  self.clock+=500;self.seq+=1;self.started=self.clock
  raise ValueError('goal_changed')
 def test_old_wait_does_not_credit_new_goal_after_replacement(self):
  with patch('recovery_budget.time.monotonic',lambda:self.clock):
   with self.assertRaises(ValueError):self.replaced_goal()
  self.clock+=4
  self.assertEqual(self.clock-self.started,4.)
 def test_old_escape_does_not_charge_new_goal_after_replacement(self):
  with patch('recovery_budget.time.monotonic',lambda:self.clock):
   with self.assertRaises(ValueError):self.replaced_goal(escape=True)
  self.clock+=4
  self.assertEqual(self.clock-self.started,4.)
 @paused_budget
 def wait_then_escape_at_limit(self):
  self.clock+=500
  with charged_motion(self):
   self.clock+=.5;require_execution_budget(self);self.moves.append('first')
   self.clock+=.51;require_execution_budget(self);self.moves.append('over_limit')
 def test_only_remaining_execution_budget_is_available_to_escape(self):
  self.clock+=479;self.moves=[]
  with patch('recovery_budget.time.monotonic',lambda:self.clock):
   with self.assertRaisesRegex(ValueError,'goal_timeout'):self.wait_then_escape_at_limit()
  self.assertEqual(self.moves,['first'])
