"""Battery task session lifecycle with fake controller only."""
import tempfile
import threading
import time
import unittest
from pathlib import Path

import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))

from battery_task import BatteryTask
from test_control import Controller, FakeAdapter, configured, Rejected


class _Flow:
    def __init__(self, store, gate, result=None):
        self.store=store;self.gate=gate
        self.result=result or {'status':'needs_attention','reason':'fixture'}

    def run(self):
        self.gate.wait(2)
        return dict(self.result)


class _Depth:
    def __init__(self):self.requested={'d435':False,'d405':True};self.calls=[]
    def snapshot(self):
        return {'cameras':{camera:{'requested':enabled} for camera,enabled in self.requested.items()}}
    def request(self,data):
        self.calls.append(dict(data));self.requested[data['camera']]=data['enabled']


class BatteryTaskTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.adapter=FakeAdapter();self.controller=Controller(configured(),self.adapter)
        self.gate=threading.Event();self.created=[]
        def factory(store,controller,token,environment_clear):
            self.created.append((store,controller,token,environment_clear))
            return _Flow(store,self.gate)
        self.task=BatteryTask(Path(self.temp.name),self.controller,factory)
        self.addCleanup(self.task.close)

    def wait(self,status):
        end=time.monotonic()+2
        while time.monotonic()<end:
            state=self.task.snapshot('owner')
            if state['status']==status:return state
            time.sleep(.01)
        self.fail('task did not reach '+status)

    def test_start_is_idempotent_and_reserves_without_hardware_calls(self):
        request={'action':'start','request_id':'req-1','expected_epoch':0,
                 'environment_clear':True}
        first=self.task.action('owner',request)
        again=self.task.action('owner',request)
        self.assertEqual(first['run_id'],again['run_id'])
        self.assertEqual(len(self.created),1)
        self.assertEqual(self.adapter.calls,[])
        self.assertTrue(self.controller.snapshot()['motion_reserved'])
        self.assertEqual(self.task.snapshot('other')['status'],'owned_by_other')
        self.created[0][0].event('tool_result',{'name':'observe_battery','ok':False,
            'error':{'code':'target_not_found','message':'没有找到电池'}})
        view=self.task.snapshot('owner')
        self.assertEqual(view['phase'],'starting')
        self.assertEqual(view['mission_stage'],'IDLE')
        self.assertEqual(view['logs'][-1]['data']['error']['code'],'target_not_found')
        with self.assertRaises(Rejected):
            self.task.action('other',{'action':'start','request_id':'req-2','expected_epoch':1,
                                      'environment_clear':True})
        self.gate.set();done=self.wait('needs_attention')
        self.assertEqual(done['reason'],'fixture')
        self.assertFalse(self.controller.snapshot()['motion_reserved'])

    def test_cancel_is_idempotent_and_marks_run_without_motion(self):
        self.task.action('owner',{'action':'start','request_id':'req-1','expected_epoch':0,
                                  'environment_clear':True})
        cancel={'action':'cancel','request_id':'cancel-1'}
        one=self.task.action('owner',cancel);two=self.task.action('owner',cancel)
        self.assertEqual(one['run_id'],two['run_id'])
        self.assertTrue((self.created[0][0].path/'cancel.flag').exists())
        self.assertEqual(self.adapter.calls,[])
        self.gate.set();self.wait('needs_attention')

    def test_invalid_payload_stale_epoch_and_other_client_cancel_are_rejected(self):
        for payload in ({}, {'action':'start','request_id':'x'},
                        {'action':'start','request_id':'x','expected_epoch':True},
                        {'action':'start','request_id':'x','expected_epoch':0,
                         'environment_clear':False},
                        {'action':'unknown','request_id':'x'}):
            with self.subTest(payload=payload),self.assertRaises((ValueError,Rejected)):
                self.task.action('owner',payload)
        with self.assertRaises(Rejected):
            self.task.action('owner',{'action':'start','request_id':'x','expected_epoch':9,
                                      'environment_clear':True})
        self.task.action('owner',{'action':'start','request_id':'good','expected_epoch':0,
                                  'environment_clear':True})
        with self.assertRaises(Rejected):
            self.task.action('other',{'action':'cancel','request_id':'stop'})
        self.gate.set();self.wait('needs_attention')

    def test_unknown_or_interrupted_completion_keeps_control_reserved(self):
        def factory(store,controller,token,environment_clear):
            class Unsafe:
                def run(inner):
                    controller.interrupt_motion(token)
                    return {'status':'needs_attention','reason':'execution_unknown'}
            return Unsafe()
        task=BatteryTask(Path(self.temp.name)/'unsafe',self.controller,factory)
        self.addCleanup(task.close)
        task.action('owner',{'action':'start','request_id':'unsafe','expected_epoch':0,
                             'environment_clear':True})
        end=time.monotonic()+2
        while task.snapshot('owner')['status']=='running' and time.monotonic()<end:time.sleep(.01)
        state=task.snapshot('owner')
        self.assertTrue(state['recovery_required'])
        self.assertTrue(self.controller.snapshot()['motion_reserved'])

    def test_interrupted_task_with_verified_stop_returns_control(self):
        for index in range(3):self.controller.update_body(.5,[0.]*4,0.,0.,0.)
        self.controller.update_arm('left',[0.]*6,-3.,[0.]*6)
        self.controller.update_arm('right',[0.]*6,-3.,[0.]*6)
        def factory(store,controller,token,environment_clear):
            class Stopped:
                def run(inner):
                    with controller.lock:controller.motion_interrupted=True;controller.command_epoch+=1
                    store.state['pending']={'last_receipt':{'status':'unknown','data':{'recovery_required':True}}}
                    store.save()
                    return {'status':'needs_attention','reason':'execution_unknown'}
            return Stopped()
        task=BatteryTask(Path(self.temp.name)/'stopped',self.controller,factory)
        self.addCleanup(task.close)
        task.action('owner',{'action':'start','request_id':'stopped','expected_epoch':0,
                             'environment_clear':True})
        end=time.monotonic()+2
        while task.snapshot('owner')['status']=='running' and time.monotonic()<end:time.sleep(.01)
        state=task.snapshot('owner')
        self.assertFalse(state['recovery_required'])
        self.assertFalse(self.controller.snapshot()['motion_reserved'])
        self.assertFalse(self.controller.snapshot()['motion_interrupted'])
        self.assertIn('recovered_pending',task.active['store'].state)

    def test_log_view_is_bounded_and_ignores_partial_records(self):
        self.task.action('owner',{'action':'start','request_id':'logs','expected_epoch':0,
                                  'environment_clear':True})
        store=self.created[0][0]
        for index in range(105):store.event('tool_result',{'stage':index})
        with (store.path/'events.jsonl').open('a') as handle:handle.write('{partial')
        logs=self.task.snapshot('owner')['logs']
        self.assertEqual(len(logs),99)
        self.assertEqual(logs[0]['data']['stage'],6)
        self.assertEqual(logs[-1]['data']['stage'],104)
        self.gate.set();self.wait('needs_attention')

    def test_task_keeps_needed_depth_on_then_restores_previous_state(self):
        depth=_Depth()
        task=BatteryTask(Path(self.temp.name)/'depth',self.controller,
            lambda store,controller,token,environment_clear:_Flow(store,self.gate),depth_control=depth)
        self.addCleanup(task.close)
        task.action('owner',{'action':'start','request_id':'depth','expected_epoch':0,
                             'environment_clear':True})
        self.assertEqual(depth.requested,{'d435':True,'d405':True})
        self.gate.set()
        end=time.monotonic()+2
        while task.snapshot('owner')['status']=='running' and time.monotonic()<end:time.sleep(.01)
        self.assertEqual(depth.requested,{'d435':False,'d405':True})
        self.assertEqual(depth.calls,[{'camera':'d435','enabled':True},{'camera':'d405','enabled':True},
                                      {'camera':'d435','enabled':False},{'camera':'d405','enabled':True}])


if __name__=='__main__':unittest.main()
