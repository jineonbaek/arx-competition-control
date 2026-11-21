import tempfile,threading,time,unittest,numpy as np
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
try:from t2_timeout_recovery import T2TimeoutRecovery
except ModuleNotFoundError:T2TimeoutRecovery=None
class Backend:
 def __init__(self,events,ready=True):self.events=events;self.ready=ready;self.thread=None;self.driver='driver';self.motion=NS(snapshot_paths=lambda:[]);self.state=dict(run_id='original')
 def get_preobservation_goal(self):return dict(joints_rad=[0.]*6)
 def snapshot(self):return self.state
 def start_preobservation_owned(self,token,goal,history,**kw):
  self.events.append('arm');self.state=dict(run_id='recovery',status='complete' if self.ready else 'held',preobservation_ready=self.ready,reason='arm unavailable');return self.state
 def action(self,data):self.events.append('arm_stop')
class RecoveryOrder(unittest.TestCase):
 def setUp(self):self.assertIsNotNone(T2TimeoutRecovery)
 def exercise(self,ready):
  events=[];b=Backend(events,ready);module=NS(placement=b,runs={'one':dict(arm_run_id='original')});ctrl=NS(motion_valid=lambda _:True,snapshot=lambda:dict(arms={'right':dict(fresh=True,joints=[0.]*6)}));ctx=dict(request_id='one',token=object(),controller=ctrl,cancelled=threading.Event())
  service=T2TimeoutRecovery(module,Path('/tmp'))
  def base(context,run,goal,guard):guard();events.append('base');return dict(status='completed')
  service._return_chassis=base;service.start(ctx,dict(point_id='point_2',goal_pose=[1.,2.,0.]),b.get_preobservation_goal());service.runs['one']['thread'].join(2)
  return events,service.inspect(ctx)
 def test_right_arm_finishes_before_any_base_movement(self):
  events,result=self.exercise(True);self.assertEqual(events,['arm','base']);self.assertEqual(result['status'],'completed');self.assertTrue(result['right_arm_ready']);self.assertTrue(result['chassis_ready'])
 def test_arm_failure_never_moves_base_or_continues(self):
  events,result=self.exercise(False);self.assertNotIn('base',events);self.assertEqual(result['status'],'failed');self.assertFalse(result['chassis_ready'])
class OwnedArmMode(unittest.TestCase):
 def setUp(self):
  import fast_visual_level as f
  self.f=f;self.assertTrue(callable(getattr(f.FastVisualLevel,'start_preobservation_owned',None)),'owned preobservation-only mode missing')
 def test_recovery_mode_preserves_gripper_and_normal_start_resets_it(self):
  f=self.f;ctrl=NS(motion_valid=lambda _:True,snapshot=lambda:dict(chassis_active=False));arm=f.FastVisualLevel(ctrl,None);seen=[]
  arm._run=lambda:seen.append(getattr(arm,'preobservation_only',False))
  with tempfile.TemporaryDirectory() as d,patch.object(f,'WORK',Path(d)),patch.object(f,'load_chassis_policy',return_value={'handoff_max_age_s':5}),patch.object(f,'validate_locked_handoff',return_value={'admission_source':'t2_current_position'}):
   arm.start_preobservation_owned(object(),dict(joints_rad=[0.]*6),[]);arm.thread.join(1);self.assertEqual(seen,[True]);self.assertFalse(arm.state['released'])
   arm.start_owned(object(),{});arm.thread.join(1);self.assertEqual(seen,[True,False])
if __name__=='__main__':unittest.main()
class RecoveryGuard(unittest.TestCase):
 def test_joint_only_recovery_does_not_depend_on_failed_camera(self):
  import ast,math,json
  source=Path(__file__).resolve().parents[1]/'src/fast_visual_level.py'
  tree=ast.parse(source.read_text());method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='guard')
  calls=[];ns=dict(np=np,required_cameras=lambda mode:(calls.append(mode) or []),require_gripper_retained=lambda *a:None,require_closed_target=lambda *a:None)
  exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)
  state=dict(arms={s:dict(fresh=True,joints=[0.]*6,gripper=0.) for s in ('left','right')},body=dict(fresh=True,height=.18),chassis_active=False)
  obj=NS(ctrl=NS(snapshot=lambda:state,motion_valid=lambda _:True,adapter=NS(trajectory_status=lambda _:dict(driver_instance='d',ready=True,enabled=True))),cancel=threading.Event(),token=object(),left_q=np.zeros(6),driver='d',initializing=False,released=False,closing_clamp=False,state={},raw=0.,mode='auto_insert_160',preobservation_only=True,auto_insert_160=True,d435_only=False,body_fixed={},inserting=False,motion=NS(active=None))
  self.assertTrue(ns['guard'](obj));self.assertEqual(calls,[])
  obj.preobservation_only=False;self.assertTrue(ns['guard'](obj));self.assertEqual(calls,['auto_insert_160'])
