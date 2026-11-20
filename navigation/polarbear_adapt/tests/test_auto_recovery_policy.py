import unittest
from urllib.error import HTTPError
from retained_goal import recovery_kind
from special_runtime import transient_input
class AutoPolicy(unittest.TestCase):
 def test_temporary_gateway_errors_are_rechecked_but_authentication_is_not_retried(self):
  for code in (502,503,504):
   e=HTTPError('http://isolated.invalid/api/state',code,'temporary',{},None)
   self.assertEqual(recovery_kind(e),'console');self.assertTrue(transient_input(e))
  e=HTTPError('http://isolated.invalid/api/state',403,'forbidden',{},None)
  self.assertEqual(recovery_kind(e),'park');self.assertFalse(transient_input(e))
if __name__=='__main__':unittest.main()
