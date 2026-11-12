"""Offline Agent planning from frozen visual evidence; no hardware or model workers."""
import copy
import hashlib
import io
import json
import tempfile
import time
import unittest
from unittest.mock import patch
import numpy as np
from agent.config import Config
from agent.storage import RunStore, atomic_json
from agent.battery_tools import BatteryExecutionTools
from agent.grasp_scene import save_frame, CALIBRATION
from agent.grasp_geometry import PROFILE, preview_geometry
from agent.tests.test_grasp_integration import frame
from web.src.handeye_column import ColumnModel

class BatteryPickPlanTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.store=RunStore.create(self.temp.name);self.addCleanup(self.store.close)
        self.model=ColumnModel();self.q=np.radians([0,20,40,10,0,-90]);self.lift=12.
        wrist=self.model.wrist_transform(self.q,self.lift)
        tcp=np.eye(4);tcp[:3,3]=[.12,.007,-.004];tcp[:3,:3]=wrist[:3,:3].T
        self.setup=dict(calibration_id='synthetic-test-only',parent_frame='right_link26',
            tool_frame='grip_reference',translation_unit='m',T_wrist_from_tool=tcp.tolist(),
            alignment_tolerance_deg=5.,j6_tolerance_deg=5.,timing_limits=dict(
                joint_speed_deg_s=[5.7]*6,joint_acceleration_deg_s2=[12.]*6,
                tcp_speed_mm_s=20.,tcp_acceleration_mm_s2=40.))
        self.store.state['mission'].update(stage='COARSE_ALIGNED',evidence_barriers={
            'COARSE_ALIGNED':dict(completed_at=time.time()-1,previous_observation_ids=[])})
        f=frame();f.update(camera_id=np.array('d405'),joints=self.q,lift_rad=np.array(self.lift))
        for key in ('image_time','arm_time','body_time'):f[key]=np.array(time.time())
        self.frame=f
        blob=io.BytesIO();np.savez(blob,**f)
        self.obs=save_frame(self.store,blob.getvalue(),source='synthetic',
            calibration_sha256=hashlib.sha256(CALIBRATION.read_bytes()).hexdigest())
        self.ident=self.obs['observation_id']
        self.store.state.update(latest_battery_observations={'d435':'head','d405':self.ident},
            battery_observations={})
        for camera,ident in self.store.state['latest_battery_observations'].items():
            path=self.store.path/(ident+'.json')
            atomic_json(path,dict(observation_id=ident,camera=camera,status='localized'))
            self.store.state['battery_observations'][ident]=path.name
        self.T=np.eye(4)
        self.T[:3,:3]=[[0,0,-1],[1,0,0],[0,-1,0]]
        target=(wrist@tcp)[:3,3]+[.01,0,0]
        grip=np.array(json.loads(PROFILE.read_text())['points_object_mm']['grip'])/1000
        self.T[:3,3]=target-self.T[:3,:3]@grip
        self.pose=dict(id=str(f['id']),frame='base_link',T_camera_from_object=self.T.tolist(),
            T_base_from_object=self.T.tolist())
        self.report=dict(observation_id=self.ident,status='preview_complete',message='fixture',
            quality=dict(version=1,status='consistent'),pose=self.pose,
            camera_handoff=dict(head_observation_id='head',wrist_observation_id=self.ident,
                quality=dict(version=1,status='consistent')),
            geometry=preview_geometry(f,np.ones((20,30),bool),self.pose,10))
        self.tools=BatteryExecutionTools(Config(),self.store,observations=object())

    def preview(self,report=None,call='preview'):
        def worker(store,ident,clearance,output,*args,**kwargs):
            output.mkdir(parents=True,exist_ok=True)
            atomic_json(output/'result.json',report or self.report)
            return report or self.report
        with patch('agent.grasp_tools.pipeline',side_effect=worker):
            result=self.tools.execute('preview_battery_grasp',
                dict(observation_id=self.ident,clearance_mm=10),call)
        self.assertTrue(result['ok'],result)
        return result

    def plan(self,call='plan'):
        return self.tools.execute('plan_battery_pick',dict(observation_id=self.ident),call)

    def test_tcp_pending_keeps_contact_line_but_never_emits_wrist_targets(self):
        self.preview()
        with patch('agent.battery_pick_plan.ColumnModel') as unused:
            result=self.plan()
        self.assertTrue(result['ok'],result)
        data=result['data'];self.assertEqual(data['status'],'tcp_pending')
        self.assertFalse(data['executable']);self.assertIsNone(data['joint_waypoints_rad'])
        self.assertIsNone(data['wrist_end_mm']);unused.assert_not_called()
        self.assertEqual(self.store.state['mission']['stage'],'COARSE_ALIGNED')
        self.assertTrue((self.store.path/data['result_file']).is_file())

    def test_cross_camera_handoff_quality_is_advisory_but_identity_stays_required(self):
        self.tools=BatteryExecutionTools(Config(),self.store,observations=object(),pick_setup=self.setup)
        report=copy.deepcopy(self.report);report['camera_handoff']['quality']['status']='rejected'
        self.preview(report,'advisory');result=self.plan('advisory_plan')
        self.assertTrue(result['ok'],result)
        self.assertIn('handoff_advisory',result['data'])
        report=copy.deepcopy(self.report);report['camera_handoff']['head_observation_id']='other'
        self.preview(report,'identity');self.assertFalse(self.plan('identity_plan')['ok'])

    def test_real_ik_uses_frozen_actual_joints_and_full_calibrated_transform(self):
        self.tools=BatteryExecutionTools(Config(),self.store,observations=object(),pick_setup=self.setup)
        self.preview();result=self.plan();self.assertTrue(result['ok'],result)
        data=result['data'];self.assertEqual(data['status'],'kinematic_preview')
        self.assertFalse(data['executable']);self.assertFalse(data['environment_collision_checked'])
        self.assertEqual(data['observation_id'],self.ident)
        self.assertEqual(data['calibration_id'],'synthetic-test-only')
        tcp=np.array(self.setup['T_wrist_from_tool'])
        for q,point in zip(data['joint_waypoints_rad'],data['tool_points_mm']):
            actual=self.model.wrist_transform(q,self.lift)@tcp
            self.assertLess(np.linalg.norm(actual[:3,3]*1000-point),.5)
            np.testing.assert_allclose(actual[:3,:3],np.eye(3),atol=.003)
            self.assertLess(abs(np.degrees(q[5])+90),5.)
        self.assertEqual(data['timing']['interpolation'],'segment_quintic_scalar')

    def test_unreviewed_or_changed_evidence_cannot_plan(self):
        self.preview();path=self.store.path/self.store.state['grasp_preview']
        path.write_text('{}')
        self.assertFalse(self.plan()['ok'])
        self.assertNotIn('battery_pick_plan',self.store.state)
        for key in ('quality','camera_handoff','geometry'):
            report=copy.deepcopy(self.report)
            if key=='geometry':report[key]['profile_sha256']='changed'
            elif key=='quality':report[key]['status']='rejected'
            else:report[key]['head_observation_id']='different-target'
            self.preview(report,call='preview_'+key)
            self.assertFalse(self.plan(call='plan_'+key)['ok'])
            self.assertNotIn('battery_pick_plan',self.store.state)

    def test_changed_archive_calibration_and_missing_feedback_cannot_plan(self):
        self.preview()
        from agent.grasp_scene import frame_path
        archive=frame_path(self.store,self.ident);original=archive.read_bytes()
        archive.write_bytes(b'changed')
        result=self.plan('changed_frame');self.assertFalse(result['ok'])
        self.assertEqual(result['error']['code'],'scene_changed')
        archive.write_bytes(original)
        cal=self.store.path/'other-calibration.json';cal.write_text('{}')
        with patch('agent.battery_pick_plan.CALIBRATION',cal):
            self.assertFalse(self.plan('changed_calibration')['ok'])
        f=copy.deepcopy(self.frame);f.pop('joints')
        buf=io.BytesIO();np.savez(buf,**f);archive.write_bytes(buf.getvalue())
        manifest=archive.parent/'manifest.json';obs=json.loads(manifest.read_text())
        obs['scene_sha256']=hashlib.sha256(buf.getvalue()).hexdigest();atomic_json(manifest,obs)
        self.assertFalse(self.plan('missing_joints')['ok'])
        self.assertNotIn('battery_pick_plan',self.store.state)

    def test_new_snapshot_stage_and_llm_supplied_tcp_are_rejected(self):
        self.preview()
        self.store.state['latest_battery_observations']['d405']='new'
        self.assertFalse(self.plan()['ok'])
        self.store.state['latest_battery_observations']['d405']=self.ident
        self.store.state['mission']['stage']='HEAD_LOCALIZED'
        self.assertFalse(self.plan('wrong_stage')['ok'])
        self.store.state['mission']['stage']='COARSE_ALIGNED'
        result=self.tools.execute('plan_battery_pick',dict(observation_id=self.ident,
            T_wrist_from_tool=np.eye(4).tolist()),'injected')
        self.assertFalse(result['ok'])
        self.assertIsNone(self.store.state['pending'])

    def test_retry_failure_discards_previous_plan_and_preview(self):
        self.preview();self.assertTrue(self.plan()['ok'])
        with patch('agent.grasp_tools.pipeline',side_effect=ValueError('pose failed')):
            result=self.tools.execute('preview_battery_grasp',dict(observation_id=self.ident,clearance_mm=10),'retry')
        self.assertFalse(result['ok'])
        for key in ('grasp_preview','grasp_preview_sha256','battery_pick_plan'):
            self.assertNotIn(key,self.store.state)
        self.assertFalse(self.plan('after_failure')['ok'])

    def test_bad_setup_wrong_j6_and_incompatible_battery_orientation_fail(self):
        self.preview()
        for key,value in [('translation_unit','mm'),('parent_frame','body4'),
                          ('T_wrist_from_tool',np.zeros((4,4)).tolist()),
                          ('alignment_tolerance_deg',float('nan')),('j6_tolerance_deg',float('inf'))]:
            setup=copy.deepcopy(self.setup);setup[key]=value
            self.tools=BatteryExecutionTools(Config(),self.store,observations=object(),pick_setup=setup)
            self.assertFalse(self.plan('bad_'+key)['ok'])
        self.tools=BatteryExecutionTools(Config(),self.store,observations=object(),pick_setup=self.setup)
        report=copy.deepcopy(self.report);report['geometry']['grip']['bump_down_error_deg']=180.
        self.preview(report,'tilted');self.assertFalse(self.plan('tilted_plan')['ok'])
        self.preview(call='restore')
        with patch('agent.battery_pick_plan.plan_insert',return_value=dict(status='kinematic_preview',
                joint_waypoints_rad=[[0,0,0,0,0,np.pi/2]],executable=False)):
            self.assertFalse(self.plan('wrong_j6')['ok'])

if __name__=='__main__':unittest.main()
