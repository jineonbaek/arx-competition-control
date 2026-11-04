"""ROS subscriber/command adapter. Does not load SDKs or launch drivers."""
import os
import threading
import time
import json
import uuid


class RosAdapter:
    def __init__(self, config):
        self.config = config
        self.node = None
        self.controller = None
        self.error = ""
        self.publishers = {}
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.pending_lift = None
        self.lift_client = None
        self.feedback_sources = {}
        self.body_calibration = None
        self.stop_lift_client = None
        self.arm_readiness = {}
        self.arm_stop_clients = {}
        self.pending_arm_stops = {}
        self.pending_arm_commands = {}

    def start(self, controller):
        self.controller = controller
        self.thread.start()

    def _run(self):
        try:
            import rclpy
            from rclpy.qos import qos_profile_sensor_data
            from arx5_arm_msg.msg import RobotStatus, RobotCmd
            from arm_control.msg import PosCmd
            from rcl_interfaces.srv import SetParameters
            from std_msgs.msg import Bool, String
            from std_srvs.srv import Trigger
            self.Trigger = Trigger
            rclpy.init()
            self.rclpy, self.RobotCmd, self.PosCmd = rclpy, RobotCmd, PosCmd
            self.SetParameters = SetParameters
            self.node = rclpy.create_node("arx_web_observer")
            for side in ("left", "right"):
                def callback(msg, side=side):
                    self.controller.update_arm(side, list(msg.joint_pos[:6]), float(msg.joint_pos[6]), list(msg.end_pos))
                self.node.create_subscription(RobotStatus, self.config["arms"][side]["state_topic"], callback, qos_profile_sensor_data)
                if self.config["arms"][side].get("command_result_topic"):
                    def command_result(msg):
                        try:
                            result = json.loads(msg.data)
                            if not isinstance(result, dict):
                                return
                            pending = self.pending_arm_commands.get(result.get("request_id"))
                            if pending:
                                pending[1].update(result)
                                pending[0].set()
                        except (ValueError, TypeError):
                            pass
                    self.node.create_subscription(String, self.config["arms"][side]["command_result_topic"], command_result, 10)
                if self.config["arms"][side].get("ready_topic"):
                    def ready_callback(msg, side=side):
                        self.arm_readiness[side] = (bool(msg.data), time.monotonic())
                    self.node.create_subscription(Bool, self.config["arms"][side]["ready_topic"], ready_callback, qos_profile_sensor_data)
                if self.config["arms"][side].get("stop_service"):
                    self.arm_stop_clients[side] = self.node.create_client(Trigger, self.config["arms"][side]["stop_service"])
            def body_callback(msg):
                self.controller.update_body(float(msg.height), list(msg.temp_float_data[1:5]),
                                            float(msg.temp_float_data[0]), float(msg.head_yaw), float(msg.head_pit))
            self.node.create_subscription(PosCmd, self.config["body"]["state_topic"], body_callback, qos_profile_sensor_data)
            calibrated_topic = self.config["body"].get("calibrated_topic")
            if calibrated_topic:
                def calibrated_callback(msg):
                    self.body_calibration = (bool(msg.data), time.monotonic())
                self.node.create_subscription(Bool, calibrated_topic, calibrated_callback, qos_profile_sensor_data)
            self.lift_client = self.node.create_client(SetParameters, self.config["body"]["lift_parameter_service"])
            if self.config["body"].get("lift_stop_service"):
                self.stop_lift_client = self.node.create_client(Trigger, self.config["body"]["lift_stop_service"])
            self.controller.log("ROS 2 观察节点已连接；未启动硬件驱动")
            next_feedback_check = 0.0
            while not self.stop_event.is_set():
                rclpy.spin_once(self.node, timeout_sec=0.05)
                self.controller.tick()
                if time.monotonic() >= next_feedback_check:
                    self.check_feedback()
                    next_feedback_check = time.monotonic() + 1.0
        except Exception as e:
            self.error = str(e)
            if self.controller:
                self.controller.log("ROS 不可用：" + self.error, "warning")
        finally:
            if self.node:
                self.node.destroy_node()
                self.node = None
            if hasattr(self, "rclpy") and self.rclpy.ok():
                self.rclpy.shutdown()


    def check_feedback(self):
        sources = {}
        specs = [(side, "左臂" if side == "left" else "右臂",
                  self.config["arms"][side]["state_topic"], "arx5_arm_msg/msg/RobotStatus")
                 for side in ("left", "right")]
        specs.append(("body", "机体", self.config["body"]["state_topic"], "arm_control/msg/PosCmd"))
        for key, label, topic, expected in specs:
            with self.controller.lock:
                state = self.controller.body if key == "body" else self.controller.arms[key]
                fresh = self.controller.fresh(state)
                received = state is not None
            try:
                endpoints = self.node.get_publishers_info_by_topic(topic)
                publishers = sum(endpoint.topic_type == expected for endpoint in endpoints)
                other = len(endpoints) - publishers
                status = ("fresh" if fresh else "type_mismatch" if not publishers and other else
                          "no_publisher" if not publishers else "stale" if received else "waiting")
                descriptions = {"fresh": "实时反馈有效", "no_publisher": "未发现反馈发布者",
                                "type_mismatch": "发布者消息类型不匹配", "stale": "反馈已过期",
                                "waiting": "已有发布者，等待有效反馈"}
                message = "%s反馈：%s（%s；有效发布者 %d）" % (label, descriptions[status], topic, publishers)
            except Exception:
                publishers, other, status = None, None, "graph_error"
                message = "%s反馈：暂时无法查询 ROS 发布者（%s）" % (label, topic)
            item = {"topic": topic, "expected_type": expected, "publishers": publishers,
                    "other_publishers": other, "status": status, "message": message}
            old = self.feedback_sources.get(key, {})
            if any(old.get(field) != item[field] for field in ("status", "publishers", "other_publishers")):
                self.controller.log(message, "info" if status == "fresh" else "warning")
            sources[key] = item
        self.feedback_sources = sources

    def status(self):
        return {"connected": self.node is not None and not self.error, "error": self.error,
                "domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
                "feedback_sources": self.feedback_sources.copy()}

    def has_subscriber(self, topic, kind):
        if not self.node or self.error:
            return False
        expected = "arx5_arm_msg/msg/RobotCmd" if kind == "arm" else "arm_control/msg/PosCmd"
        try:
            return any(x.topic_type == expected for x in self.node.get_subscriptions_info_by_topic(topic))
        except Exception:
            return False

    def arm_ready(self, side):
        ready = self.arm_readiness.get(side)
        return ready is not None and ready[0] and time.monotonic() - ready[1] < 1.0

    def arm_stop_ready(self, side):
        client = self.arm_stop_clients.get(side)
        return client is not None and client.service_is_ready()

    def arm_stop(self, side):
        from control import Rejected
        pending = self.pending_arm_stops.get(side)
        if pending is not None and not pending.done():
            raise Rejected("Previous arm stop request is pending")
        pending = self.arm_stop_clients[side].call_async(self.Trigger.Request())
        self.pending_arm_stops[side] = pending
        def completed(future):
            try:
                result = future.result()
                self.controller.log("%s arm stop: %s" % (side, result.message), "info" if result.success else "warning")
            except Exception as error:
                self.controller.log("%s arm stop failed: %s" % (side, error), "warning")
        pending.add_done_callback(completed)

    def body_calibrated(self):
        return self.body_calibration is not None and self.body_calibration[0] and time.monotonic() - self.body_calibration[1] < 1.0

    def lift_ready(self):
        return self.node is not None and not self.error and self.lift_client is not None and self.lift_client.service_is_ready()

    def lift_stop_ready(self):
        return self.stop_lift_client is not None and self.stop_lift_client.service_is_ready()

    def lift_stop(self):
        from control import Rejected
        if self.pending_lift is not None and not self.pending_lift.done():
            raise Rejected("上一条升降请求尚未返回")
        self.pending_lift = self.stop_lift_client.call_async(self.Trigger.Request())
        def completed(future):
            try:
                result = future.result()
                self.controller.log("升降停止接口：" + result.message, "info" if result.success else "warning")
            except Exception as error:
                self.controller.log("升降停止接口异常：" + str(error), "warning")
        self.pending_lift.add_done_callback(completed)

    def _publisher(self, topic, msg_type):
        if topic not in self.publishers:
            self.publishers[topic] = self.node.create_publisher(msg_type, topic, 1)
        return self.publishers[topic]

    def arm(self, side, joints, gripper, tcp, mode, kind=None, wait=True):
        msg = self.RobotCmd()
        msg.joint_pos, msg.gripper, msg.end_pos, msg.mode = joints, float(gripper), tcp, mode
        if kind is None:
            self._publisher(self.config["arms"][side]["command_topic"], self.RobotCmd).publish(msg)
            return
        from control import Rejected
        request_id = str(uuid.uuid4())
        msg.header.frame_id = kind + "|" + request_id
        pending = (threading.Event(), {})
        self.pending_arm_commands[request_id] = pending
        try:
            self._publisher(self.config["arms"][side]["command_topic"], self.RobotCmd).publish(msg)
        except Exception:
            self.pending_arm_commands.pop(request_id, None)
            raise
        def confirmed():
            try:
                if not pending[0].wait(1.0):
                    raise Rejected("机械臂驱动未返回确认；执行结果不确定，请查看反馈")
                if pending[1].get("ok") is not True:
                    raise Rejected(pending[1].get("message", "机械臂驱动拒绝指令"))
            finally:
                self.pending_arm_commands.pop(request_id, None)
        return confirmed() if wait else confirmed

    def chassis(self, x, y, yaw, body):
        msg = self.PosCmd()
        msg.height = body["height"]
        msg.temp_float_data[0] = body["waist"]
        msg.head_yaw, msg.head_pit = body["head_yaw"], -body["head_pitch"]
        # Dedicated body input remaps the vendor VR callback; never use the shared left-arm topic.
        scale = self.config["body"].get("chassis_input_scale", 2.5)
        msg.chx, msg.chy, msg.chz = x * scale, -y * scale, yaw * scale
        msg.mode1 = 2 if (x, y, yaw) == (0, 0, 0) else 1
        self._publisher(self.config["body"]["command_topic"], self.PosCmd).publish(msg)

    def lift(self, target):
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        if self.pending_lift is not None and not self.pending_lift.done():
            from control import Rejected
            raise Rejected("上一条升降请求尚未返回")
        request = self.SetParameters.Request()
        request.parameters = [Parameter(name=self.config["body"]["lift_parameter_name"],
                              value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(target)))]
        self.pending_lift = self.lift_client.call_async(request)
        def completed(future):
            try:
                result = future.result()
                if not result or not result.results or not all(x.successful for x in result.results):
                    self.controller.log("升降请求被驱动拒绝：" + "; ".join(x.reason for x in result.results) if result else "升降驱动未返回", "warning")
                else:
                    self.controller.log("驱动已接受升降目标；尚未确认实际到位")
            except Exception as e:
                self.controller.log("升降服务异常：" + str(e), "warning")
        self.pending_lift.add_done_callback(completed)

    def close(self):
        if self.controller:
            with self.controller.lock:
                self.controller._stop_chassis("网页服务退出，已请求停车")
        self.stop_event.set()
        self.thread.join(timeout=2)
