import threading,unittest
from pathlib import Path
from t2_preview import T2PreviewModule
from recorded_t2_planner import RecordedT2Planner

class TestRecorded(unittest.TestCase):
    def test_recorded_d435_reaches_t2_adapter_without_completion(self):
        root=Path("/home/arx/arx-place-hold-20261116");run=root/"reports/d435-visual-level-active/estimate-1794782014072959960"
        planner=RecordedT2Planner(frame=run/"frame.npz",pose=run/"pose.json",workspace=root,control_config="/home/arx/arx-competition-control/web/config/control.json")
        task=type("ForbidLive",(),{"start_owned":lambda self,t:(_ for _ in ()).throw(AssertionError("hardware"))})()
        module=T2PreviewModule(planner,task);ctx={"run_id":"competition-A","stage":"place","request_id":"competition-A:place","token":object(),"cancelled":threading.Event()}
        self.assertEqual(module.start(ctx)["status"],"running");self.assertTrue(module.wait(ctx,15.));result=module.inspect(ctx)
        self.assertEqual(result["status"],"failed");self.assertFalse(result["preview"]["executable"])
        self.assertEqual(result["preview"]["stage_order"],["cabinet","level","align","insert"])
        self.assertEqual(result["preview"]["insert"]["status"],"kinematic_preview")
        self.assertEqual(result["preview"]["d405"]["status"],"unavailable")
if __name__=="__main__":unittest.main()
