"""Competition entry forwards the deployed Plan F defaults without robot IO."""
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock,patch

from battery_task import BatteryTask
from competition_modules import GraspModule
from agent.battery_production import workflow_factory


class CompetitionPlanFTests(unittest.TestCase):
    def test_competition_reuses_preload_profile_and_plan_f_factory(self):
        root=Path(__file__).resolve().parents[2]
        factory=workflow_factory(root/'config/battery-execution.json',
            console_url='http://127.0.0.1:9983',gripper_limits=(-3.14,0.))
        self.assertEqual(factory.task_settings['vision_mode'],'plan_f')
        self.assertEqual(factory.task_settings['lift_target_rad'],12.52)
        self.assertEqual(factory.task_settings['lift_acceptance_rad'],12.)
        controller=Mock();controller.motion_valid.return_value=True
        controller.snapshot.return_value={'command_epoch':0}
        depth=Mock();depth.snapshot.return_value={'cameras':{
            'd435':{'requested':False},'d405':{'requested':False}}}
        session=Mock()
        with tempfile.TemporaryDirectory() as directory:
            task=BatteryTask(Path(directory)/'runs',controller,factory,depth_control=depth)
            profile={'id':'custom-test','name':'fixture','dino_prompt':'fixture handle','qwen_task':'fixture battery'}
            task.task_settings['recognition_profile']=profile.copy()
            grasp=GraspModule(task)
            with patch('battery_task.PerceptionSession',return_value=session),patch('agent.battery_production.build_workflow') as build,patch.object(task,'_run'):
                try:
                    grasp.prewarm(threading.Event())
                    task.prepared['worker'].join(1)
                    session.prewarm.assert_called_once();build.assert_not_called()
                    prepared=task.prepared['store']
                    grasp.preposition_lift(controller,'competition-token')
                    controller.submit_competition_lift.assert_called_once_with(
                        {'type':'lift','target':12.52},'competition-token')
                    task.start_owned('motion-token',True)
                    task.active['worker'].join(1)
                    self.assertIs(task.active['store'],prepared)
                    self.assertIs(prepared.perception_session,session)
                    self.assertEqual(prepared.state['battery_task_settings']['recognition_profile'],profile)
                    self.assertEqual(build.call_args.kwargs['vision_mode'],'plan_f')
                    coarse=build.call_args.kwargs['policies']['coarse']
                    self.assertEqual((coarse['fixed_lift_target_rad'],coarse['lift_acceptance_rad']),(12.52,12.))
                    self.assertEqual(depth.request.call_args_list[0].args[0],{'camera':'d405','enabled':True})
                    self.assertFalse(any(call.args[0].get('camera')=='d435' for call in depth.request.call_args_list))
                finally:
                    task.close()


if __name__=='__main__':unittest.main()
