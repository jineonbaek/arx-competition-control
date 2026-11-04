READ-ONLY REAL DEVICE VERIFICATION - latest result

Verified 15 feedback channels: left/right 6 arm motors each, two gripper motors,
and lift motor. Each channel returned three measured positions. No motor enable,
disable, movement setpoint, parameter write, save, clear-error or zero command
was sent. Official high-level motor controllers were not instantiated.

Repeat a finite real-device read (takes about 45 seconds, ports must be free):
  bash /home/arx/arx-competition-control/scripts/read_state_once.sh

This runs command-boundary tests, only requests documented parameters/status,
records raw CAN replies and serial writes, closes the adapters and checks all
writes against finite allowlists. The result is a sampled snapshot, not a live
stream and not a synchronized snapshot. An in-progress result is invalidated.

Latest result: reports/live-readonly-status.json
Raw replies: reports/readonly-status-query.json, readonly-encos-query.json
Audit logs: reports/readonly-*-query-syscalls.log
Protocol source records: reports/readonly-protocol-sources.json

All arm angles are raw motor feedback before robot joint sign/offset mapping.
Gripper angle is not calibrated jaw width. Lift feedback is a total motor angle
in rad, not metres. SDK getter sign is opposite the lift motor angle. No zeroing
or homing was done, so no calibrated physical height is claimed.

The earlier zero-frame probe results below are historical: motors did not
spontaneously report while idle. Documented queries obtain actual feedback.

--- Setup and historical investigation notes ---

ARX competition control environment - independent offline setup

Location: /home/arx/arx-competition-control
Protected existing directories: /home/arx/LIFT, /home/arx/ROS2_LIFT_Play,
/home/arx/docs_lift2. This setup does not write to or load SDKs from them.

Official sources (sparse checkouts; revisions recorded in reports/sources.json):
- X5: Python arm SDK, branch master
- ARX_X5: ROS 2 arm drivers/messages, branch main
- LIFT: Python and ROS 2 platform SDK, branch main

Ubuntu 24.04, Python 3.12, system ROS 2 Jazzy and compiler tools are reused.
The new .venv inherits system Python packages for ROS compatibility and contains
its own pybind11 2.13.6. Nothing is installed globally. Build artifacts, SDK
installation, logs and reports all remain within this directory.

Activate (does not start devices):
  source /home/arx/arx-competition-control/env.sh

Repeat offline verification:
  bash /home/arx/arx-competition-control/scripts/check.sh

Rebuild and stage compiled Python modules:
  bash /home/arx/arx-competition-control/scripts/build.sh
  python3 /home/arx/arx-competition-control/scripts/stage_python.py

Verification imports SDKs without constructing arm, platform, or encoder objects.
It checks API availability, pose conversion, offline forward kinematics, ROS
message serialization, package resolution, dynamic dependencies and repository
cleanliness. System-call tracing checks for CAN sockets and ARX/serial device
access. No robot node or DDS publisher is started. No CAN interface is started.

Compatibility correction:
Official ARX_X5 arm_control/package.xml omits the ament_cmake build_type export.
Colcon otherwise identifies it as a catkin package and omits its ament prefix
hook. Only workspace/src/arm_control/package.xml is corrected in a copied
package. Original official checkouts remain clean. Exact patch is recorded in
reports/arm_control-jazzy.patch.

Hardware controls have NOT been validated. This environment includes no
hardware launch shortcut. Do not execute vendor demos as offline tests:
they can initialize motors, home the arms, set lift height or publish commands.
Coordinates, height units, end-effector type and command semantics must be
confirmed before any later hardware initialization or movement.

Reports:
  reports/build.json
  reports/sources.json
  reports/offline-check.json
  reports/no-hardware-access.json
  reports/offline-syscalls.log
  logs/build-*.log

Verified result: all 3 build jobs passed; 4 ROS 2 packages built; 11/11 offline checks passed.
No CAN/serial device access was found in the traced offline check.
Vendor KDL warning: base_link inertia is ignored by KDL; this check proves only
that forward kinematics returns finite values, not model accuracy or gravity compensation.

Read-only hardware probe: scripts/passive_can_capture.py opens the three
USB serial adapters exclusively and only sends adapter-local C, S8, L
commands (close, bitrate 1 Mbps, listen-only request). It never sends
CAN data/remote frames or normal-open O. After 12 seconds it sends C and
closes each port. Current result: 0 bytes/frames on all three adapters;
listen-only was not acknowledged, so actual mode and robot communication
are unverified. Missing feedback is represented as null, never zero.
Results: reports/live-readonly-status.json and passive-can-capture.json.
Offline command guard tests: python3 scripts/test_passive_capture.py.

Follow-up: all three USB adapter version queries succeeded (5f97fef).
Receive-only capture sends adapter-local C,S8,O,C commands only; O opens
the adapter channel, not a motor. No CAN data or remote frames were sent.
All buses returned 0 bytes in 12 seconds. Exact writes are saved in
reports/receive-only-can-writes.log and receive-only-adapter-writes.txt.
Static disassembly confirms the LiftHeadControlLoop constructor calls
MotorType3::packEnableMotor and SocketCan::ExchangeData four times.
This constructor was NOT executed; its read() path was not used.
Robot was reported powered on with emergency stop released.
Joint, gripper and lift states remain unavailable/null. Further work
needs a verified feedback query path that does not enable motors.

Calibration conventions audit (no motion):
  python3 scripts/audit_calibration_conventions.py  # offline, no device access
Read reports/calibration-conventions.txt and calibration-conventions.json.
The unverified calibration profile intentionally leaves physical zero offsets,
gripper width and absolute lift height unset. No active control configuration
was changed. Note Python lift command scale differs from ROS2 raw-angle input.

2026-11-03 标定资料追查：reports/calibration-source-followup.txt；reports/calibration-followup-status.json；官方模型/零位图/原文件只读副本位于references/calibration-followup/。本轮无硬件指令，物理标定仍未验证。

升降完整链路核查：reports/lift-unit-chain/analysis.txt；status.json；loader-resolution.json。仅静态分析和加载器依赖列表，无控制程序启动。

2026-11-03 用户授权并现场急停监看：已启动原机体程序，日志报告升降/腰部校准，用户确认停稳无异常；PID9380持续以fixed_body_height=0.0运行，ROS_DOMAIN_ID=191；can5由本轮slcand维持。没有发送最高位命令。记录reports/original-body-calibration/result.json。

2026-11-03 用户指明460mm为物理上限：原机体程序以19.1084rad目标启动校准后上升。现场确认已到最高、已按急停并停稳；随后PID9431控制程序已退出，无自动重启。记录reports/lift-to-460/result.json。此前PID9380零位控制进程已退出。can5通信适配器仍开启，急停保持按下。

最高限位记录：config/lift-upper-limit-observed.json；reports/lift-upper-limit-record.txt。现场确认上止挡约18.85rad、标称460mm。19.1084rad只是先前目标，不是实测限位；尚未启用软件上限或自动运动。

2026-11-03 上限位置复测：原程序重新校准后以18.3rad目标运行，反馈约18.040rad，连续20秒稳定，用户确认“已停稳，无异常，顶部有间隙”。PID10222仍在保持该目标，ROS_DOMAIN_ID=191。记录reports/lift-softlimit-repeat/result.json；未重复碰触止挡。

用户报告上升太快且有异响：低速候选在lift-low-speed/，已编译和离线检查，目标斜坡0.5rad/s、回零0.5rad/s。尚未启动，未接管PID10222；异响原因仍未确认。

主机重启后只读检查：无控制进程自动启动，电机未使能，用户确认最低点。当前PID10387等旧运行状态已失效；以reports/current-runtime-state.json为准。零点和累计圈数需重新核对，停走问题未解决，本轮无运动命令。报告reports/post-reboot-readonly/result.json。

重启后重新低速升高完成：低速控制器PID4011保持18.3rad目标，反馈约18.028rad，现场确认停稳、无异响或抖动。停走连续性问题尚未声称解决。以reports/current-runtime-state.json为当前状态；本轮记录reports/reboot-low-speed-rise/result.json。原工程文件未改。

低速连续性继续排查：reports/continuity-comparison/findings.txt。两次记录呈约0.393rad空间重复停顿；有限上下对比发现方向相关位置偏差。当前PID4011保持18.3rad，未增加力矩/增益；待核对电机铭牌。SDK活动时禁止并发寄存器查询，原解析器会混淆帧。

低位双向诊断完成：18.3→2→5→2rad，原速度/增益/力矩不变。PID4011现保持2rad目标，反馈约2.429rad；分析reports/low-bidirectional-diagnostic/analysis.json。已补足上下运动记录，但尚未应用拟合参数，连续性问题未解决。
