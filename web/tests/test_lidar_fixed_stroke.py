"""Direct lidar placement: measured travel, no camera dependency or seating claim."""
import json
from pathlib import Path
import shutil
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np
import mouth_push
from insertion_contact import InsertionContact
import test_d405_horizontal_flow as horizontal_fixtures
import test_navigation_placement as navigation_fixtures


class DistanceStrokeTests(unittest.TestCase):
    def exercise(self, distances, contact=False):
        start=np.eye(4);angle=np.radians(8.)
        start[:3,:3]=[[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]]
        actual=start.copy();moves=[];remaining=iter(distances)
        def move(phase,length,rotation):
            moves.append(length)
            actual[:3,3]+=next(remaining)*start[:3,0]/1000.
            if contact:raise InsertionContact(dict(stop_confirmed=True,reference_reset=True,kind='progress_stall'))
        result=mouth_push.run_distance_stroke(start,lambda:actual.copy(),move,lambda:True)
        return result,moves,start,actual

    def test_one_axial_stroke_completes_without_visual_evidence(self):
        result,moves,start,actual=self.exercise([150.])
        self.assertEqual(moves,[150.]);self.assertTrue(result['release_ready'])
        self.assertFalse(result['insertion_depth_verified'])
        self.assertNotIn('verified_depth_mm',result)
        mouth_push.require_distance_release(start,actual,start[:3,0],result)

    def test_small_tracking_shortfall_has_only_one_bounded_finish(self):
        result,moves,_,_=self.exercise([149.8,.45])
        self.assertEqual(len(moves),2);self.assertAlmostEqual(moves[1],.45)
        self.assertTrue(result['release_ready'])

    def test_large_shortfall_or_contact_cannot_release_or_repeat_push(self):
        for contact in (False,True):
            result,moves,_,_=self.exercise([130.],contact)
            self.assertFalse(result['release_ready']);self.assertEqual(moves,[150.])

    def test_two_short_strokes_do_not_loop(self):
        result,moves,_,_=self.exercise([149.5,.1])
        self.assertFalse(result['release_ready']);self.assertEqual(len(moves),2)

    def test_wrong_direction_nonfinite_and_excess_travel_fail(self):
        for distance in (-5.,float('nan'),155.):
            with self.assertRaises(ValueError):self.exercise([distance])

    def test_cancelled_dispatch_never_returns_release_ready(self):
        def stop():raise ValueError('cancelled')
        with self.assertRaisesRegex(ValueError,'cancelled'):
            mouth_push.run_distance_stroke(np.eye(4),lambda:np.eye(4),lambda *a:self.fail('motion'),stop)

    def test_release_rechecks_actual_travel_after_task_receipt(self):
        result,_,start,actual=self.exercise([150.])
        actual[:3,3]-=.002*start[:3,0]
        with self.assertRaises(ValueError):mouth_push.require_distance_release(start,actual,start[:3,0],result)


class AutomaticRouteTests(unittest.TestCase):
    def exercise(self,mode,source):
        level=horizontal_fixtures.D405HorizontalFlowTests().make_level()
        self.addCleanup(shutil.rmtree,level.run)
        level.placement_mode=mode
        level.owned_handoff=dict(admission_source=source)
        level.reached_route=[level.ctrl.q.tolist()]
        snapshot=level.ctrl.snapshot
        level.ctrl.snapshot=lambda:dict(snapshot(),body=dict(height=level.height))
        saved=dict(preobserve_joints_deg=np.degrees(level.ctrl.q).tolist(),preobserve_source='saved',tolerance_mm=1.,insert_speed_mm_s=30.)
        level.saved_observation=lambda:saved
        from agent.taught_placement import PhaseTimeline
        level.timeline=PhaseTimeline()
        events=[]
        def timed(phase,fn,**kw):events.append(phase);return fn()
        level._timed=timed;level.guard=lambda:True
        def trim(*args):events.append('camera_trim');return dict(converged=True)
        level.auto_d405_horizontal_trim=trim
        def push(model,tcp,axis,length,**kw):
            events.append('push');level.ctrl.q[1]+=length/1000.
        level.execute_locked_axis=push
        def verify(*args):
            events.append('camera_seat')
            return dict(insertion_depth_verified=True,verified_depth_mm=150.,handle_outside_mm=45.,verification_frame_ids=['fresh'],verification_image_time=time.time())
        level.check_fixed_depth=verify
        def release(check):check(level.ctrl.snapshot());events.append('release_return');level.save(released=True)
        level.release_and_return=release
        with patch('fast_visual_level.validate_preobservation_pose',return_value={}),patch('fast_visual_level.correct_observation_sag',side_effect=lambda *a:events.append('j4')):
            level.auto_insert_160_place()
        return level.state,events

    def test_lidar_and_t2_skip_both_vision_stages_keep_point_j4_push_return(self):
        for source in ('lidar_point_2','t2_current_position'):
            state,events=self.exercise('lidar_fixed_stroke',source)
            self.assertNotIn('camera_trim',events);self.assertNotIn('camera_seat',events)
            self.assertLess(events.index('j4'),events.index('push'))
            self.assertLess(events.index('push'),events.index('release_return'))
            self.assertEqual(state['status'],'complete');self.assertTrue(state['returned'])
            self.assertFalse(state['insertion_depth_verified']);self.assertTrue(state['d405_skipped'])

    def test_visual_entry_and_visual_mode_keep_existing_checks(self):
        for mode,source in [('visual_seated','lidar_point_2'),('lidar_fixed_stroke','visual_chassis')]:
            state,events=self.exercise(mode,source)
            self.assertIn('camera_trim',events);self.assertIn('camera_seat',events)
            self.assertTrue(state['insertion_depth_verified'])


class NavigationReceiptTests(unittest.TestCase):
    def fixture(self):
        fx=navigation_fixtures.NavigationPlacementTests();fx.setUp();self.addCleanup(fx.doCleanups)
        fx.module.lidar_locked=True;fx.arm.placement_mode='lidar_fixed_stroke'
        fx.arm.snapshot=lambda:dict(status='complete',run_id='arm-1',released=True,returned=True,
            completion_basis='measured_axial_travel',seat_verification_enabled=False,
            insertion_depth_verified=False,minimum_travel_reached=True,measured_travel_mm=150.,release_ready=True)
        return fx

    def test_navigation_accepts_distance_receipt_without_claiming_visual_seating(self):
        fx=self.fixture();fx.module.start(fx.context);result=fx.wait()
        self.assertEqual(result['status'],'completed',result)
        self.assertFalse(result['placement_confirmed'])
        self.assertEqual(result['completion_basis'],'measured_axial_travel')

    def test_short_or_unreturned_receipt_cannot_finish(self):
        for changes in (dict(measured_travel_mm=149.),dict(returned=False),dict(released=False),dict(completion_basis='visual_seated')):
            fx=self.fixture();original=fx.arm.snapshot;fx.arm.snapshot=lambda:original()|changes
            fx.module.start(fx.context);self.assertEqual(fx.wait()['status'],'failed')

    def test_readiness_no_longer_requires_seated_reference_file(self):
        fx=self.fixture();ready=fx.module.readiness()
        self.assertTrue(ready['reference_valid'],ready);self.assertFalse(ready['seat_verification_enabled'])

    def test_timed_wrapper_returns_distance_completion_without_waiting_for_timeout(self):
        from types import SimpleNamespace
        from t2_timed_placement import TimedT2Placement
        fx=self.fixture();context=fx.context|dict(controller=fx.ctrl)
        wrapper=TimedT2Placement(fx.module,fx.nav,fx.root,recovery=SimpleNamespace(capture_goal=lambda:None))
        wrapper.start(context);fx.wait();result=wrapper.inspect(context)
        self.assertEqual(result['status'],'completed',result)
        self.assertNotEqual(result.get('outcome'),'timed_out')
        self.assertEqual(result['completion_basis'],'measured_axial_travel')


class PolicyTests(unittest.TestCase):
    def test_absent_config_keeps_visual_mode_and_invalid_mode_fails(self):
        from placement_policy import load_placement_mode
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'config').mkdir()
            self.assertEqual(load_placement_mode(root),'visual_seated')
            path=root/'config/t2-placement.json'
            path.write_text(json.dumps(dict(version=1,mode='lidar_fixed_stroke')))
            self.assertEqual(load_placement_mode(root),'lidar_fixed_stroke')
            path.write_text(json.dumps(dict(version=1,mode='typo')))
            with self.assertRaises(ValueError):load_placement_mode(root)


class PolicyHTTPTests(unittest.TestCase):
    def test_both_automatic_entrypoints_advertise_no_visual_wait_and_do_not_move(self):
        import threading
        from http.server import ThreadingHTTPServer
        from types import SimpleNamespace
        from navigation_placement import NavigationPlacement,CurrentPositionPlacement
        from server import make_handler
        from test_control import Controller,FakeAdapter,configured
        from test_battery_task_http import BatteryTaskHTTPTests
        adapter=FakeAdapter();config=configured();config['chassis_backend']='local'
        ctrl=Controller(config,adapter)
        arm=SimpleNamespace(placement_mode='lidar_fixed_stroke')
        module=NavigationPlacement(ctrl,None,arm,None,Path('/unused'),lidar_locked=True)
        single=CurrentPositionPlacement(ctrl,arm,Path('/unused'))
        competition=SimpleNamespace(modules={'place':module},snapshot=lambda *a:dict(scope='competition',status='idle',tasks=[]))
        t2=SimpleNamespace(modules={'place':single},snapshot=lambda *a:dict(scope='t2_test',status='idle',tasks=[]))
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(ctrl,competition_task=competition,t2_test_task=t2))
        server.daemon_threads=True;server.stopping=threading.Event()
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        self.addCleanup(thread.join);self.addCleanup(server.server_close);self.addCleanup(server.shutdown);self.addCleanup(server.stopping.set)
        request=BatteryTaskHTTPTests.request
        _,headers,_=request(server,'GET','/t2-test.html')
        auth={'Cookie':headers['Set-Cookie'].split(';')[0]}
        for endpoint in ('t2-test','competition-task'):
            code,_,body=request(server,'GET','/api/'+endpoint,headers=auth)
            self.assertEqual(code,200)
            view=json.loads(body);policy=view['insertion_policy']
            self.assertFalse(policy['visual_verification']);self.assertFalse(policy['d405_horizontal_enabled'])
            self.assertEqual(policy['minimum_measured_travel_mm'],150.)
            self.assertTrue(view['placement_readiness']['reference_valid'])
        self.assertEqual(adapter.calls,[])
