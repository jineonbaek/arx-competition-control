import time,threading,unittest
from agent.insertion_verification import run_verified_insertion


def sample(depth,index=1):
    return dict(depth_mm=depth,frame_id=str(index),image_time=time.time(),calibration_sha256='a'*64,
                axis_base=[1.,0.,0.],up_base=[0.,0.,1.])

class VerificationTests(unittest.TestCase):
    def run_case(self,depths,**kwargs):
        moves=[];events=[];items=iter(sample(d,i+1) for i,d in enumerate(depths))
        result=run_verified_insertion(lambda reference:next(items),lambda distance,reference:moves.append(distance),
            guard=lambda:None,event=lambda **v:events.append(v),**kwargs)
        return result,moves,events

    def test_outside_gap_is_added_once_and_partial_insertion_subtracted(self):
        result,moves,_=self.run_case([-25.,150.])
        self.assertEqual(moves,[175.]);self.assertTrue(result['insertion_depth_verified'])
        self.assertEqual(result['initial_gap_mm'],25.)
        result,moves,_=self.run_case([20.,149.])
        self.assertEqual(moves,[130.]);self.assertTrue(result['insertion_depth_verified'])

    def test_visual_shortfall_gets_measured_supplement(self):
        result,moves,events=self.run_case([-10.,142.,150.])
        self.assertEqual(moves,[160.,8.]);self.assertEqual(result['compensation_count'],1)
        self.assertEqual(result['verified_depth_mm'],150.)
        self.assertIn('verify',[x['phase'] for x in events])

    def test_compensation_budget_terminates_without_faking_success(self):
        result,moves,_=self.run_case([-10.,120.,130.,140.])
        self.assertEqual(moves,[160.,10.,10.]);self.assertFalse(result['insertion_depth_verified'])
        self.assertEqual(result['status'],'insertion_unverified');self.assertEqual(result['compensation_count'],2)

    def test_no_visual_progress_and_replayed_frame_do_not_keep_pushing(self):
        result,moves,_=self.run_case([-10.,-10.])
        self.assertEqual(moves,[160.]);self.assertFalse(result['insertion_depth_verified'])
        f=sample(-10.);moves=[]
        result=run_verified_insertion(lambda ref:f,lambda d,ref:moves.append(d),guard=lambda:None)
        self.assertEqual(moves,[160.]);self.assertFalse(result['insertion_depth_verified'])

    def test_missing_initial_observation_and_out_of_range_gap_never_move(self):
        moves=[]
        def missing(ref):raise ValueError('no battery depth')
        result=run_verified_insertion(missing,lambda d,ref:moves.append(d),guard=lambda:None)
        self.assertEqual(moves,[]);self.assertFalse(result['insertion_depth_verified'])
        result,moves,_=self.run_case([-200.]);self.assertEqual(moves,[])

    def test_slow_capture_times_out_and_late_result_never_moves(self):
        done=threading.Event();moves=[]
        def slow(ref):done.wait(.3);return sample(-10.)
        started=time.monotonic()
        result=run_verified_insertion(slow,lambda d,ref:moves.append(d),guard=lambda:None,observation_timeout_s=.03)
        self.assertLess(time.monotonic()-started,.2);self.assertFalse(result['insertion_depth_verified'])
        done.set();time.sleep(.01);self.assertEqual(moves,[])

    def test_motion_errors_and_cancellation_are_not_visual_compensation(self):
        def bad_move(d,ref):raise RuntimeError('driver fault')
        with self.assertRaisesRegex(RuntimeError,'driver fault'):
            run_verified_insertion(lambda ref:sample(-10.),bad_move,guard=lambda:None)
        def cancelled():raise ValueError('cancelled')
        with self.assertRaisesRegex(ValueError,'cancelled'):
            run_verified_insertion(lambda ref:sample(-10.),bad_move,guard=cancelled)

    def test_cancel_at_observation_failure_is_not_an_unverified_finalize(self):
        cancelled=[False];go=threading.Event();calls=[0]
        def guard():
            if cancelled[0]:raise ValueError('cancelled')
            calls[0]+=1
            if calls[0]==2:go.set()
        def observe(reference):
            go.wait(1.);cancelled[0]=True;raise ValueError('capture failed')
        with self.assertRaisesRegex(ValueError,'cancelled'):
            run_verified_insertion(observe,lambda d,ref:self.fail('must not move'),guard=guard)

    def test_cancel_on_terminal_verification_event_never_returns_for_release(self):
        cancelled=[False];values=iter([sample(-10.),sample(150.,2)])
        def guard():
            if cancelled[0]:raise ValueError('cancelled')
        def event(**value):
            if value['phase']=='verified':cancelled[0]=True
        with self.assertRaisesRegex(ValueError,'cancelled'):
            run_verified_insertion(lambda ref:next(values),lambda d,ref:None,guard=guard,event=event)

    def test_old_or_changed_calibration_cannot_confirm(self):
        rows=[sample(-10.),sample(150.,2)];rows[1]['calibration_sha256']='b'*64;moves=[]
        result=run_verified_insertion(lambda ref:rows.pop(0),lambda d,ref:moves.append(d),guard=lambda:None)
        self.assertFalse(result['insertion_depth_verified']);self.assertEqual(moves,[160.])

if __name__=='__main__':unittest.main()
