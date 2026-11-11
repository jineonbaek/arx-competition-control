import json
from pathlib import Path
import shlex
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock,patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
import foundationpose_preview as fp


class FoundationPoseTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.preview=fp.FoundationPosePreview(Mock(),Mock(),self.root/"cache")
        self.addCleanup(self.preview.close)
        self.demo=self.root/"demo";self.demo.mkdir()
        self.patch=patch.object(fp,"DEMO",self.demo);self.patch.start();self.addCleanup(self.patch.stop)
        self.runner=self.demo/"run-snapshot.sh"
        self.runner.write_text("exec sleep 60")
        self.preview.capture.cache.mkdir(parents=True)
        np.savez(self.preview.capture.cache/"frame.npz",id="frozen")
        self.preview.owner="owner"
        self.preview.job=dict(status="captured",view=dict(id="frozen",width=80,height=60))

    def infer(self,client="owner",**changes):
        return self.preview.action(client,dict(op="infer",id="frozen",roi=[10,10,50,50],**changes))

    def test_reading_page_does_not_load_models_or_depth(self):
        with patch.object(fp.subprocess,"Popen") as spawn:
            self.assertFalse(self.preview.snapshot()["running"])
            spawn.assert_not_called()
        self.preview.capture.depth.acquire_grasp.assert_not_called()

    def test_invalid_roi_stale_frame_and_other_client_never_spawn(self):
        for request in [dict(op="infer",id="old",roi=[10,10,50,50]),
                        dict(op="infer",id="frozen",roi=[0,0,999,999]),
                        dict(op="infer",id="frozen",roi=[False,0,50,50]),
                        dict(op="infer",id="frozen",roi=[10,10,50,50],shell="x")]:
            with self.assertRaises(ValueError):self.preview.action("owner",request)
        with self.assertRaises(ValueError):self.infer("other")
        self.assertIsNone(self.preview.process)

    def test_cancel_reaps_process_and_rejects_other_client(self):
        state=self.infer();process=self.preview.process
        with self.assertRaises(ValueError):self.infer()
        with self.assertRaises(ValueError):self.preview.action("other",dict(op="stop",run_id=state["run_id"]))
        stopped=self.preview.action("owner",dict(op="stop",run_id=state["run_id"]))
        self.assertFalse(stopped["running"])
        self.assertIsNotNone(process.poll())
        self.preview.capture.ctrl.submit.assert_not_called()

    def test_timeout_and_shutdown_reap_process(self):
        self.infer();process=self.preview.process
        self.preview.started=time.monotonic()-100
        self.assertEqual(self.preview.snapshot()["status"],"error")
        self.assertIsNotNone(process.poll())
        self.infer();process=self.preview.process
        self.preview.close()
        self.assertIsNotNone(process.poll())

    def test_completed_job_exits_and_releases_without_browser_polling(self):
        code="import json,sys; from pathlib import Path; Path(sys.argv[1]).write_text(json.dumps("+repr(dict(id="frozen",T_base_from_object=np.eye(4).tolist()))+"))"
        self.runner.write_text("for output do :; done\nexec "+shlex.quote(sys.executable)+" -c "+shlex.quote(code)+' "$output"')
        state=self.infer();process=self.preview.process
        deadline=time.monotonic()+4
        while self.preview.process is not None and time.monotonic()<deadline:time.sleep(.05)
        self.assertIsNotNone(process.poll())
        self.assertIsNone(self.preview.process)
        self.assertEqual(self.preview.snapshot()["status"],"complete")
        self.assertEqual(self.preview.snapshot()["result"]["id"],"frozen")

    def test_failed_job_cannot_show_old_success(self):
        self.runner.write_text("exit 1")
        self.preview.job["result"]={"old":True}
        self.infer()
        deadline=time.monotonic()+4
        while self.preview.snapshot()["running"] and time.monotonic()<deadline:time.sleep(.05)
        self.assertEqual(self.preview.snapshot()["status"],"error")
        self.assertIsNone(self.preview.snapshot()["result"])

    def test_independent_capture_cache_does_not_change_handle_cache(self):
        from grasp_preview import GraspPreview,CACHE
        other=GraspPreview(Mock(),Mock())
        self.assertEqual(other.cache,CACHE)
        self.assertNotEqual(other.cache,self.preview.capture.cache)
