import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
import vision_preview
from vision_preview import VisionPreview

class VisionPreviewTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.patch=patch.object(vision_preview,"DEMO",Path(self.temp.name))
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.preview=VisionPreview(Path(self.temp.name)/"runs")
        self.addCleanup(self.preview.close)
        self.command=patch.object(self.preview,"_command",return_value=[
            sys.executable,"-c","import time; time.sleep(60)"])
        self.command.start()
        self.addCleanup(self.command.stop)

    def start(self,camera="d435"):
        return self.preview.action("owner",{"op":"start","camera":camera})

    def test_idle_never_starts_a_process(self):
        self.assertFalse(self.preview.snapshot()["running"])
        self.preview._command.assert_not_called()
        self.assertIsNone(self.preview.process)

    def test_camera_allowlist_and_fields(self):
        for data in [{"op":"start","camera":"astra"},{"op":"start","camera":"d435","command":"x"}]:
            with self.assertRaises(ValueError):self.preview.action("owner",data)
        self.preview._command.assert_not_called()

    def test_repeated_start_is_idempotent_and_stop_reaps_process(self):
        state=self.start()
        process=self.preview.process
        self.assertEqual(self.start()["pid"],process.pid)
        stopped=self.preview.action("owner",{"op":"stop","run_id":state["run_id"]})
        self.assertFalse(stopped["running"])
        self.assertIsNotNone(process.poll())
        self.assertIsNone(self.preview.frame(state["run_id"]))

    def test_other_session_cannot_renew_or_stop(self):
        state=self.start()
        for op in ["heartbeat","stop"]:
            with self.assertRaises(ValueError):
                self.preview.action("other",{"op":op,"run_id":state["run_id"]})
        self.assertTrue(self.preview.snapshot()["running"])

    def test_lease_file_and_expiry(self):
        state=self.start()
        lease=self.preview.cache/state["run_id"]/"lease.json"
        self.assertGreater(json.loads(lease.read_text())["expires_at"],time.time())
        process=self.preview.process
        self.preview.deadline=time.monotonic()-1
        deadline=time.monotonic()+4
        while self.preview.snapshot()["running"] and time.monotonic()<deadline:
            time.sleep(.1)
        self.assertIsNotNone(process.poll())
        self.assertFalse(self.preview.snapshot()["running"])

    def test_frame_identity_and_path_checks(self):
        state=self.start()
        root=self.preview.cache/state["run_id"]
        (root/"frame.jpg").write_bytes(b"jpeg")
        (root/"state.json").write_text(json.dumps({
            "sequence":4,"captured_at":time.time(),"image_file":"frame.jpg"}))
        self.assertEqual(self.preview.frame(state["run_id"]),(4,b"jpeg"))
        self.assertIsNone(self.preview.frame("unknown"))
        (root/"state.json").write_text(json.dumps({
            "sequence":5,"captured_at":time.time(),"image_file":"../../outside.jpg"}))
        self.assertIsNone(self.preview.frame(state["run_id"]))

if __name__=="__main__":unittest.main()
