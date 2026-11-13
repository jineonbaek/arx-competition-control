import unittest
from odom_guard import OdomGuard,map_output_allowed

class GuardTests(unittest.TestCase):
    def send(self,g,t,**kw):
        return g.accept(int(t*1e9),int(kw.get('now',t)*1e9),kw.get('p',[0,0,0]),
                        kw.get('q',[0,0,0,1]),kw.get('v',[0]*6))
    def test_static_gap_requires_three_fresh_samples(self):
        g=OdomGuard();self.assertEqual(self.send(g,10),'publish')
        self.assertEqual(self.send(g,11.1),'wait')
        for t in (11.2,11.3):self.assertEqual(self.send(g,t),'wait')
        self.assertEqual(self.send(g,11.4),'publish')
    def test_stale_samples_do_not_recover(self):
        g=OdomGuard();self.send(g,10);self.send(g,11.1)
        for t in (11.2,11.3,11.4):self.assertEqual(self.send(g,t,now=t+1),'wait')
        self.assertTrue(g.pending)
    def test_backward_and_duplicate_fault(self):
        for stamp in (10,9.9):
            g=OdomGuard();self.send(g,10);self.assertEqual(self.send(g,stamp),'fault')
    def test_gap_with_movement_faults(self):
        for kw in ({'p':[.021,0,0]},{'v':[.051,0,0,0,0,0]},{'q':[0,0,.02,.9998]}):
            g=OdomGuard();self.send(g,10);self.assertEqual(self.send(g,11.1,**kw),'fault')
    def test_pose_drift_during_recovery_faults(self):
        g=OdomGuard();self.send(g,10);self.send(g,11.1)
        self.assertEqual(self.send(g,11.2,p=[.03,0,0]),'fault')
    def test_initial_stale_data_is_not_published(self):
        g=OdomGuard();self.assertEqual(self.send(g,10,now=11),'wait')
        self.assertEqual(self.send(g,11.1),'publish')
    def test_recovery_helper_cannot_publish_first_sample(self):
        g=OdomGuard();g.begin_recovery(10_000_000_000,[0,0,0],[0,0,0,1])
        self.assertEqual(self.send(g,10.1),'wait')
        self.assertEqual(self.send(g,10.2),'wait')
        self.assertEqual(self.send(g,10.3),'publish')
        h=OdomGuard();h.begin_recovery(10_000_000_000,[0,0,0],[0,0,0,1])
        self.assertEqual(self.send(h,10.1,p=[.1,0,0]),'fault')
    def test_map_requires_healthy_fresh_body(self):
        self.assertTrue(map_output_allowed({'healthy':True,'last_published_ns':10_000_000_000},10_100_000_000))
        for state,now in [({'healthy':False,'last_published_ns':10_000_000_000},10_100_000_000),
                          ({'healthy':True,'last_published_ns':10_000_000_000},11_000_000_000),({},10)]:
            self.assertFalse(map_output_allowed(state,now))
    def test_measured_static_twist_noise_with_stable_pose(self):
        g=OdomGuard();self.send(g,10)
        noise={'p':[.002,.001,.004],'q':[0,0,.0005,.999999875],
               'v':[.0273,0,0,0,0,.0694]}
        self.assertEqual(self.send(g,11.1,**noise),'wait')
        self.assertEqual(self.send(g,11.2,**noise),'wait')
        self.assertEqual(self.send(g,11.3,**noise),'wait')
        self.assertEqual(self.send(g,11.4,**noise),'publish')
if __name__=='__main__':unittest.main()
