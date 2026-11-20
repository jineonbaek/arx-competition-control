import unittest
from chassis_command import WebChassis


class IdentityTransportTests(unittest.TestCase):
    def setUp(self):
        self.sent=[]
        self.t=WebChassis(submit=lambda m:self.sent.append(m) or {'ok':True})
        self.t.sdk_units=True;self.t.native_sdk=True
    def test_cleanup_is_end_not_global_stop(self):
        self.t.send((.1,0,0));handle=self.t.current
        self.t.end_output(handle)
        self.assertEqual(self.sent[-1]['type'],'chassis.end_output')
        with self.assertRaises(RuntimeError):self.t.send((.1,0,0),handle)
    def test_zero_is_a_scoped_setpoint_not_a_task_end(self):
        self.t.send((0,0,0));self.t.send((.1,0,0))
        self.assertEqual(self.sent[0]['type'],'chassis.nav_sdk')
        self.assertEqual(self.sent[0]['generation'],self.sent[1]['generation'])
    def test_old_callback_ends_only_captured_generation(self):
        old=self.t.current;self.t.end_output(old);new=self.t.begin_task()
        self.t.send((.1,0,0),new);self.t.end_output(old)
        self.assertEqual(self.t.current,new)
        self.assertEqual(self.sent[-1]['generation'],new.generation)
    def test_original_time_and_identity_are_preserved(self):
        self.t.send((.1,0,0));m=self.sent[0]
        self.assertEqual(set(m),{'type','x','y','omega','source_id','task_id','generation','seq','issued_at','ttl_s'})
        self.assertGreater(m['issued_at'],0);self.assertEqual(m['ttl_s'],.3)

if __name__=='__main__':unittest.main()
