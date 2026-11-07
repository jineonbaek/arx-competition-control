import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))

class PreviewTests(unittest.TestCase):
    def test_atomic_frames_and_offline_state_do_not_reuse_old_image(self):
        self.assertIsNotNone(importlib.util.find_spec('camera_preview'),'Camera preview publisher not implemented')
        from camera_preview import publish, select_device, SPECS
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp)
            publish(d,'d435',b'\xff\xd8frame\xff\xd9',7,100.,'online')
            self.assertEqual((d/'d435.jpg').read_bytes(),b'\xff\xd8frame\xff\xd9')
            meta=json.loads((d/'d435.json').read_text())
            self.assertEqual(meta['sequence'],7)
            self.assertEqual(meta['captured_at'],100.)
            publish(d,'d435',None,7,None,'offline')
            self.assertEqual(json.loads((d/'d435.json').read_text())['state'],'offline')
            self.assertFalse((d/'d435.jpg').exists())
        records=[dict(device='/dev/video40',product=SPECS[0]['product'],index=0,interface='4-2:1.3'),dict(device='/dev/video0',product=SPECS[0]['product'],index=0,interface='4-2:1.0')]
        self.assertEqual(select_device(SPECS[0],records),'/dev/video40')
        self.assertIsNone(select_device(SPECS[0],records+[records[0]]))
        self.assertIsNone(select_device(SPECS[0],[]))

    def test_other_camera_frames_remain_bound_to_their_metadata(self):
        from camera_preview import publish
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp)
            publish(d,'d405',b'first-frame',1,100.,'online')
            first=json.loads((d/'d405.json').read_text())['image_file']
            publish(d,'d405',b'second-frame',2,101.,'online')
            second=json.loads((d/'d405.json').read_text())['image_file']
            self.assertNotEqual(first,second)
            self.assertEqual((d/first).read_bytes(),b'first-frame')
            self.assertEqual((d/second).read_bytes(),b'second-frame')

    def test_rgbd_publisher_keeps_the_capture_owned_filename(self):
        from camera_preview import publish,RECENT
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);before=len(RECENT['d405'])
            publish(d,'d405',b'frame',1,100.,'online',extra={'image_file':'d405-v-abc123-1.jpg','depth':{'image_file':'d405-z-abc123-1.png'}})
            meta=json.loads((d/'d405.json').read_text())
            self.assertEqual(meta['image_file'],'d405-v-abc123-1.jpg')
            self.assertEqual(meta['depth']['image_file'],'d405-z-abc123-1.png')
            self.assertEqual(len(RECENT['d405']),before)

    def test_both_realsense_models_use_the_existing_rgbd_owner(self):
        from unittest.mock import patch
        from camera_preview import worker,SPECS
        with patch('battery_rgbd.run') as run:
            for spec in SPECS[:2]:
                worker(spec,object())
                self.assertEqual(run.call_args.args[0],spec)
            self.assertEqual(run.call_count,2)

    def test_single_camera_capture_can_disable_all_detection(self):
        from camera_preview import parse_options
        options=parse_options(['--camera','d405','--no-detector'])
        self.assertEqual(options.camera,'d405')
        self.assertTrue(options.no_detector)
        defaults=parse_options([])
        self.assertIsNone(defaults.camera)
        self.assertTrue(defaults.no_detector)
        import contextlib,io
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            parse_options(['--camera','unknown'])

    def test_d405_only_mode_starts_one_worker_and_never_starts_detector(self):
        import contextlib,io
        from unittest.mock import Mock,patch
        import camera_preview as preview
        with tempfile.TemporaryDirectory() as tmp:
            web=Path(tmp)/'web';web.mkdir()
            stop=Mock();stop.is_set.side_effect=[False,True]
            with patch.object(preview,'WEB',web),patch.object(preview,'OUT',web/'public/camera-live'),patch.object(preview.threading,'Event',return_value=stop),patch.object(preview.threading,'Thread') as thread,patch.object(preview.signal,'signal'),patch.object(preview.subprocess,'Popen') as launch,patch.object(preview,'publish'),patch.object(Path,'is_file',return_value=True),contextlib.redirect_stdout(io.StringIO()):
                preview.main(['--camera','d405','--no-detector'])
                self.assertEqual([call.kwargs['args'][0]['id'] for call in thread.call_args_list],['d405'])
                launch.assert_not_called()
                thread.return_value.start.assert_called_once()
                thread.return_value.join.assert_called_once()

if __name__=='__main__':unittest.main()
