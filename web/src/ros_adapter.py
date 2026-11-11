"""ROS subscriber/command adapter. Does not load SDKs or launch drivers."""
import os
import threading
import time
import json
import uuid
import copy
import math


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
        self.trajectory_states = {}
        self.trajectory_lock = threading.Lock()

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
            self.String = String
            rclpy.init()
            self.rclpy, self.RobotCmd, self.PosCmd = rclpy, RobotCmd, PosCmd
            self.SetParameters = SetParameters
            self.node = rclpy.create_node("arx_web_observer")
            for side in ("left", "right"):
                def trajectory_callback(msg, side=side):
                    try:
                        if len(msg.data)>4096:raise ValueError('Oversized trajectory status')
                        self.update_trajectory(side,json.loads(msg.data))
                    except (ValueError,TypeError):
                        self.update_trajectory(side,None)
                self.node.create_subscription(String,"/web/"+side+"/trajectory_status",trajectory_callback,1)
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
        confirmed=self._arm_confirmation(request_id,
            lambda:self._publisher(self.config["arms"][side]["command_topic"],self.RobotCmd).publish(msg))
        return confirmed() if wait else confirmed

    def _arm_confirmation(self,request_id,publish):
        from control import Rejected
        pending=(threading.Event(),{})
        self.pending_arm_commands[request_id]=pending
        try:publish()
        except Exception:
            self.pending_arm_commands.pop(request_id,None)
            raise
        def confirmed():
            try:
                if not pending[0].wait(1.0):
                    raise Rejected("机械臂驱动未返回确认；执行结果不确定，请查看反馈")
                if pending[1].get("ok") is not True:
                    raise Rejected(pending[1].get("message","机械臂驱动拒绝指令"))
            finally:
                self.pending_arm_commands.pop(request_id,None)
        return confirmed

    def update_trajectory(self,side,state):
        fields={'driver_instance','command_epoch','enabled','request_id','status','reason','ready','manual_joint_active'}
        valid=(side in ('left','right') and isinstance(state,dict) and fields<=state.keys()
            and all(type(state[k]) is bool for k in ('enabled','ready','manual_joint_active'))
            and type(state['command_epoch']) is int and 0<=state['command_epoch']<2**64
            and all(isinstance(state[k],str) and len(state[k])<=256 for k in ('driver_instance','request_id','status','reason'))
            and bool(state['driver_instance'])
            and state['status'] in ('idle','running','failed','reference_complete','disabled'))
        with self.trajectory_lock:
            if valid:self.trajectory_states[side]=({k:state[k] for k in fields},time.monotonic())
            else:self.trajectory_states.pop(side,None)

    def trajectory_status(self,side):
        with self.trajectory_lock:
            cached=self.trajectory_states.get(side)
            if not self.node or self.error or cached is None or not 0<=time.monotonic()-cached[1]<=.3:
                return None
            return copy.deepcopy(cached[0])

    def trajectory_start(self,side,plan,expected):
        """Internal transport only; caller must own motion and approve the checked scene/TCP."""
        from control import Rejected, vector, number
        state=self.trajectory_status(side)
        if (not state or not state['enabled'] or not state['ready'] or state['manual_joint_active']
                or state['status'] not in ('idle','failed','reference_complete')
                or not isinstance(expected,dict)
                or any(state[k]!=expected.get(k) for k in ('driver_instance','command_epoch'))):
            raise Rejected('轨迹驱动未就绪、状态过期或已变化')
        plan=copy.deepcopy(plan)
        if (not isinstance(plan,dict) or set(plan)!={'interpolation','time_from_start_s','joint_waypoints_rad'}
                or plan['interpolation']!='segment_quintic_scalar'):
            raise Rejected('轨迹格式无效')
        times,points=plan['time_from_start_s'],plan['joint_waypoints_rad']
        if (not isinstance(times,list) or not 2<=len(times)<=1024
                or not isinstance(points,list) or len(points)!=len(times)
                or not all(number(t) for t in times) or times[0]!=0
                or any(b<=a for a,b in zip(times,times[1:]))
                or not all(vector(q,6) for q in points)):
            raise Rejected('轨迹时间或节点无效')
        limits=self.config['arms'][side]['joint_limits_deg']
        if (not isinstance(limits,list) or len(limits)!=6
                or not all(vector(pair,2) and pair[0]<=pair[1] for pair in limits)):
            raise Rejected('轨迹关节限位配置无效')
        if any(not lo<=math.degrees(v)<=hi for q in points for v,(lo,hi) in zip(q,limits)):
            raise Rejected('轨迹关节超出配置范围')
        handle=dict(request_id=str(uuid.uuid4()),driver_instance=state['driver_instance'],command_epoch=state['command_epoch'])
        msg=self.String();msg.data=json.dumps({**plan,**handle,'kind':'start','version':1},allow_nan=False)
        if len(msg.data.encode())>262144:raise Rejected('轨迹消息过大')
        confirm=self._arm_confirmation(handle['request_id'],
            lambda:self._publisher('/web/'+side+'/trajectory',self.String).publish(msg))
        return handle,confirm

    def trajectory_keepalive(self,side,handle):
        from control import Rejected
        state=self.trajectory_status(side)
        if (not state or not state['enabled'] or not state['ready'] or state['status']!='running'
                or not isinstance(handle,dict)
                or any(state[k]!=handle.get(k) for k in ('driver_instance','request_id'))
                or type(handle.get('command_epoch')) is not int
                or state['command_epoch']!=handle['command_epoch']+1):
            raise Rejected('轨迹已停止、状态过期或任务不匹配；不续发心跳')
        msg=self.String();msg.data=json.dumps({'kind':'keepalive',
            'driver_instance':state['driver_instance'],'request_id':state['request_id']})
        self._publisher('/web/'+side+'/trajectory',self.String).publish(msg)

    def nav_chassis_sdk(self, x, y, yaw):
        msg = self.PosCmd()
        msg.chx, msg.chy, msg.chz = x, y, yaw
        msg.mode1 = 1
        self._publisher("/web/nav/sdk_command", self.PosCmd).publish(msg)

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
