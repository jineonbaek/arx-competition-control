"""Check the effective per-run profile before any model or robot operation."""
import copy
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch

from battery_task import BatteryTask,FAST_HANDLE_QWEN_TASK


class FastHandleProfileTests(unittest.TestCase):
    def test_competition_and_console_fast_use_near_references_without_changing_stable(self):
        with tempfile.TemporaryDirectory() as directory:
            controller=Mock();controller.motion_valid.return_value=True
            controller.snapshot.return_value={'command_epoch':0}
            seen=[]
            def factory(store,*args):
                seen.append(copy.deepcopy(store.state['battery_task_settings']))
                return SimpleNamespace()
            task=BatteryTask(Path(directory)/'runs',controller,factory)
            original=copy.deepcopy(task.task_settings)
            with patch('battery_task.threading.Thread'):
                for mode in ('competition_fast','fast_plan_f','stable'):
                    task.active=None
                    settings=copy.deepcopy(original)
                    if mode!='stable':settings['pickup_flow']=mode
                    task._start('test',None,True,settings,token='owned')
                    task.active['store'].close()
                    self.assertEqual(settings.get('recognition_profile'),original['recognition_profile'])
            for settings in seen[:2]:
                profile=settings['recognition_profile']
                self.assertEqual(profile['qwen_task'],FAST_HANDLE_QWEN_TASK)
                self.assertEqual(profile['reference_images'],{
                    'first':['d405-02','d405-04'],'reobserve':['d405-02','d405-04']})
                self.assertEqual(profile['dino_prompt'],original['recognition_profile']['dino_prompt'])
            self.assertEqual(seen[2],original)
            self.assertEqual(task.task_settings,original)
