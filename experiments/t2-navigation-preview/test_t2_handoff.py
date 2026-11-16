import unittest
from unittest.mock import patch
from t2_preview import T2PreviewModule
import test_competition_task
from test_navigation_session import NavigationSessionTests

class T2HandoffTest(unittest.TestCase):
    def test_real_protocol_does_not_complete_a_preview(self):
        case=NavigationSessionTests(methodName="test_total_sequence_real_handoff_never_opens_manual_between_jobs")
        case.setUp();self.addCleanup(case.doCleanups)
        original=test_competition_task.Module
        task=type("Forbidden",(),{"start_owned":lambda self,t:(_ for _ in ()).throw(AssertionError("real task called"))})()
        planner_calls=[]
        def factory(trace,name):
            if name!="place":return original(trace,name)
            return T2PreviewModule(lambda ctx:planner_calls.append((ctx["stage"],ctx["run_id"],ctx["request_id"])) or {"executable":False,"insert":{"status":"kinematic_preview"}},task)
        with patch.object(test_competition_task,"Module",side_effect=factory):
            result,trace,checked=case.run_total()
        self.assertEqual(result["status"],"needs_attention")
        self.assertEqual(len(planner_calls),1);self.assertEqual(planner_calls[0][0],"place")
        state=case.hub.state();point2=[e for e in state["events"] if e.get("point_id")=="point_2"]
        self.assertIn("accept",[e["event"] for e in point2])
        self.assertNotIn("complete",[e["event"] for e in point2])
        self.assertNotEqual(state["task"].get("outcome"),"completed")
if __name__=="__main__":unittest.main()
