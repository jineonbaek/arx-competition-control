import http.client
import json
from pathlib import Path
import sys
import threading
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from server import make_handler
from control import Controller
from http.server import ThreadingHTTPServer
from test_control import FakeAdapter, configured

class HTTPTests(unittest.TestCase):
    def setUp(self):
        config = configured()
        config["execution_enabled"] = False
        self.adapter = FakeAdapter()
        self.controller = Controller(config, self.adapter)
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(self.controller))
        self.server.daemon_threads = True
        self.server.stopping = threading.Event()
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.port = self.server.server_port
    def tearDown(self):
        self.server.stopping.set()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
    def request(self, method, path, body=None, headers=None):
        c = http.client.HTTPConnection("127.0.0.1", self.port)
        c.request(method, path, body, headers or {})
        r = c.getresponse()
        result = (r.status, dict(r.getheaders()), r.read())
        c.close()
        return result
    def test_static_state_and_traversal(self):
        code, headers, body = self.request("GET", "/")
        self.assertEqual(code, 200)
        self.assertIn("SameSite=Strict", headers["Set-Cookie"])
        self.assertIn(b"ARX", body)
        code, _, body = self.request("GET", "/api/state")
        self.assertFalse(json.loads(body)["execution_enabled"])
        self.assertEqual(self.request("GET", "/%2e%2e/config/control.json")[0], 404)
        self.assertEqual(self.adapter.calls, [])

    def test_tcp_capture_endpoint_records_feedback_without_dispatching_motion(self):
        import numpy as np
        class DeterministicWristModel:
            def wrist_transform(self, joints, lift):
                transform = np.eye(4)
                transform[:3, 3] = [0.4, -0.1, 0.5 + lift / 100.0]
                return transform
        self.controller.models["right"] = DeterministicWristModel()
        _, response_headers, _ = self.request("GET", "/tcp-calibration.html")
        headers = {"Content-Type": "application/json",
                   "Cookie": response_headers["Set-Cookie"].split(";")[0],
                   "Origin": "http://127.0.0.1:%d" % self.port}
        self.controller.update_body(6.0, [0.0] * 4, 0.0, 0.0, 0.0)
        self.controller.update_arm("right", [0.0] * 6, -1.0, [0.0] * 6)

        code, _, body = self.request("GET", "/api/tcp-calibration", headers={"Cookie": headers["Cookie"]})
        self.assertEqual(code, 200)
        self.assertEqual(json.loads(body)["arms"]["right"]["sample_count"], 0)

        code, _, body = self.request(
            "POST", "/api/tcp-calibration",
            json.dumps({"op": "capture", "arm": "right", "confirmed": True}), headers,
        )
        self.assertEqual(code, 200, body)
        self.assertEqual(json.loads(body)["arms"]["right"]["sample_count"], 1)
        self.assertEqual(self.adapter.calls, [])
    def test_origin_and_observation_guard(self):
        _, headers, _ = self.request("GET", "/")
        cookie = headers["Set-Cookie"].split(";")[0]
        body = json.dumps({"type": "arm.stop", "arm": "left"})
        headers = {"Content-Type": "application/json", "Cookie": cookie, "Origin": "http://evil.invalid"}
        self.assertEqual(self.request("POST", "/api/command", body, headers)[0], 403)
        headers["Origin"] = "http://127.0.0.1:%d" % self.port
        self.assertEqual(self.request("POST", "/api/command", body, headers)[0], 400)
        self.assertEqual(self.adapter.calls, [])

    def test_mode_endpoint_blocks_observation_then_dispatches_full_message_to_fake_adapter(self):
        _, response_headers, _ = self.request("GET", "/")
        headers = {"Content-Type": "application/json",
                   "Cookie": response_headers["Set-Cookie"].split(";")[0],
                   "Origin": "http://127.0.0.1:%d" % self.port}
        for mode in range(4):
            body = json.dumps({"type": "arm.mode", "arm": "right", "mode": mode})
            self.assertEqual(self.request("POST", "/api/command", body, headers)[0], 400)
        self.assertEqual(self.adapter.calls, [])
        self.controller.config["execution_enabled"] = True
        self.controller.config["arms"]["right"]["verified_modes"] = [0, 1, 2, 3]
        self.controller.update_arm("right", [.2] * 6, -1.2, [.3] * 6)
        for mode in range(4):
            body = json.dumps({"type": "arm.mode", "arm": "right", "mode": mode})
            status, _, payload = self.request("POST", "/api/command", body, headers)
            self.assertEqual(status, 200)
            self.assertEqual(json.loads(payload)["mode_status"], "requested_unconfirmed")
            self.assertEqual(self.adapter.calls[-1],
                             ("arm", ("right", [.2] * 6, -1.2, [.3] * 6, mode)))

    def test_vehicle_wrist_target_to_joint_driver(self):
        import numpy as np
        from unittest.mock import patch
        from scipy.spatial.transform import Rotation
        _, response, _ = self.request("GET", "/")
        headers={"Cookie":response["Set-Cookie"].split(";")[0], "Content-Type":"application/json",
                 "Origin":f"http://127.0.0.1:{self.port}"}
        c=self.controller
        c.config["execution_enabled"]=True
        for stamp in (10.,10.1,10.2,10.3):
            c.clock=lambda: stamp
            c.update_body(6., [0.]*4, 0., 0., 0.)
        for side in ("left","right"):
            q=np.radians([0,12,18,4,-2,3])
            c.update_arm(side,q.tolist(),-1.,[0.]*6)
            model=c.models[side]
            goal=model.pose(q+np.radians([1,2,-3,1,2,1]),6.)[1]
            target=dict(type="arm.wrist",frame="base_link",arm=side,position_mm=(goal[:3,3]*1000).tolist(),
                        orientation_deg=Rotation.from_matrix(goal[:3,:3]).as_euler("xyz",degrees=True).tolist())
            def post(data):
                code,_,body=self.request("POST","/api/command",json.dumps(data),headers)
                return code,json.loads(body)
            # Exercise the production ACK contract without a ROS or hardware process.
            c.config["arms"][side]["command_result_topic"]="/mock/result"
            commands=[]
            def arm(*args,**kwargs):
                commands.append((args,kwargs))
                return lambda: None
            with patch.object(self.adapter,"arm",arm):
                code,body=post(target)
                self.assertEqual(code,200,body)
                args,kwargs=commands[0]
                self.assertEqual((args[0],kwargs["kind"],args[4],args[2]),(side,"arm.joints",5,-1.))
                actual=model.pose(args[1],6.)[1]
                self.assertLess(np.linalg.norm(actual[:3,3]-goal[:3,3]),.0005)
                err=Rotation.from_matrix(actual[:3,:3].T @ goal[:3,:3]).magnitude()
                self.assertLess(err,np.radians(.2))
                for invalid in [dict(target,frame="body4"),dict(target,position_mm=[5000,0,0]),
                                dict(target,position_mm=[10**1000,0,0])]:
                    self.assertEqual(post(invalid)[0],400)
                with patch.object(model,"check",return_value={"clear":False}):
                    self.assertEqual(post(target)[0],400)
                with patch.object(self.adapter,"body_calibrated",return_value=False):
                    self.assertEqual(post(target)[0],400)
                    self.assertIsNone(c.snapshot()["arms"][side]["base_wrist"])
                if side == "right":
                    c.update_body(7., [0.]*4, 0., 0., 0.)
                    self.assertEqual(post(target)[0],400)
                self.assertEqual(len(commands),1)

    def test_chassis_vector_reaches_adapter_once_and_keeps_limits_and_brake(self):
        _, response_headers, _ = self.request("GET", "/")
        headers = {"Content-Type": "application/json",
                   "Cookie": response_headers["Set-Cookie"].split(";")[0],
                   "Origin": "http://127.0.0.1:%d" % self.port}
        self.controller.config["execution_enabled"] = True
        self.controller.config["body"].update(max_linear_m_s=2, max_angular_rad_s=1)
        self.controller.update_body(5., [0.] * 4, .2, .1, -.1)

        def post(data):
            return self.request("POST", "/api/command", json.dumps(data), headers)[0]

        axis = 2 ** .5
        move = {"type": "chassis", "x": axis, "y": axis, "omega": 1, "sequence": 1}
        self.assertEqual(post(move), 200)
        self.assertEqual(len(self.adapter.calls), 1)
        self.assertEqual(self.adapter.calls[-1][1][:3], (axis, axis, 1))
        for invalid in ({"x": 2, "y": 2}, {"omega": 1.01}, {"x": True},
                        {"x": 10 ** 1000}, {"x": 0, "y": 0, "omega": 0},
                        {"x": float("nan")}, {"direction": "forward"}):
            self.assertEqual(post({**move, **invalid, "sequence": 2}), 400)
        self.assertEqual(len(self.adapter.calls), 1)
        self.assertEqual(post({"type": "chassis.stop", "sequence": 3}), 200)
        self.assertEqual(self.adapter.calls[-1][1][:3], (0, 0, 0))
        self.assertEqual(post({**move, "sequence": 2}), 400)
        self.assertEqual(len(self.adapter.calls), 2)


    def test_grasp_scene_routes_cameras_and_keeps_origin_guard(self):
        from unittest.mock import patch
        import grasp_preview
        _,response,_=self.request('GET','/')
        headers={'Cookie':response['Set-Cookie'].split(';')[0],
                 'Content-Type':'application/json','Origin':f'http://127.0.0.1:{self.port}'}
        seen=[]
        def capture(owner,data):
            seen.append(owner.camera)
            return {'id':'frozen','frame_id':'base_link'}
        with patch.object(grasp_preview.GraspPreview,'handle_action',capture):
            for camera in ('d435','d405'):
                self.assertEqual(self.request('POST','/api/grasp-scene',json.dumps({'camera':camera}),headers)[0],200)
            for invalid in ({'camera':'astra'},{'camera':'d435','op':'move'},[],{}):
                self.assertEqual(self.request('POST','/api/grasp-scene',json.dumps(invalid),headers)[0],400)
            self.assertEqual(self.request('POST','/api/grasp-scene','{"camera":"d435"}',dict(headers,Origin='http://evil.invalid'))[0],403)
        self.assertEqual(seen,['d435','d405'])
        self.assertEqual(self.adapter.calls,[])

    def test_handle_preview_survives_without_candidate_endpoint(self):
        from unittest.mock import patch
        import grasp_preview
        code, response, page = self.request("GET", "/handle.html")
        self.assertEqual(code, 200)
        self.assertIn(b"js/handle.js", page)
        headers={"Cookie":response["Set-Cookie"].split(";")[0],
                 "Content-Type":"application/json", "Origin":f"http://127.0.0.1:{self.port}"}
        frozen={"id":"saved", "frame_id":"base_link", "points":[[.4,-.25,.7,0,0,0]]}
        with patch.object(grasp_preview.GraspPreview,"latest",return_value=frozen):
            code,_,body=self.request("GET","/api/handle-preview",headers=headers)
            self.assertEqual(code,200)
            self.assertEqual(json.loads(body)["points"],frozen["points"])
        for path in ("/grasp.html","/js/grasp.js","/api/grasp-preview"):
            self.assertEqual(self.request("GET",path,headers=headers)[0],404)
        self.assertEqual(self.request("POST","/api/grasp-preview",'{"op":"infer"}',headers)[0],404)
        self.assertEqual(self.request("POST","/api/handle-preview",'{"op":"infer"}',headers)[0],400)
        home=self.request("GET","/")[2]
        self.assertNotIn(b'href="grasp.html"',home)
        self.assertIn(b'href="handle.html"',home)
        self.assertEqual(self.adapter.calls,[])

if __name__ == "__main__": unittest.main()
