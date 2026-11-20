import unittest
from run_coarse import run_cycles
class Tests(unittest.TestCase):
 def test_replans_until_arrived(self):
  statuses=iter(['bounded_correction_arrived','outside_arrived']);calls=[]
  result=run_cycles(lambda: calls.append(1) or {'status':next(statuses)},4)
  self.assertEqual(result['status'],'outside_arrived');self.assertEqual(len(calls),2)
 def test_held_is_not_retried(self):
  calls=[];result=run_cycles(lambda:calls.append(1) or {'status':'held'},4)
  self.assertEqual(result['status'],'held');self.assertEqual(len(calls),1)
 def test_budget_does_not_claim_arrival(self):
  result=run_cycles(lambda:{'status':'bounded_correction_arrived'},2)
  self.assertEqual(result['status'],'cycle_limit')
 def test_unknown_status_stops(self):
  with self.assertRaises(ValueError):run_cycles(lambda:{'status':'unknown'},2)
if __name__=='__main__':unittest.main()
