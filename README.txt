方舟无限比赛工作区
==================

运行位置为车载 ROG 的 /home/arx/arx-competition-control。这里维护方舟无限
双臂、底盘、升降的网页控制，以及独立的导航适配。日常使用从桌面启动器进入。

目录结构
--------

  web/
    public/            9983 网页、样式和交互逻辑
    src/               Python HTTP 后端、控制校验和 ROS 2 接口
    config/            控制配置、反馈话题、设备限位及来源
    hardware/          X5 双臂与机体驱动源码和启动脚本
    tests/             网页与后端的软件测试
    start.sh           只启动网页服务
    README.md          网页、驱动和限位说明，含历史接入记录
  scripts/
    start-console.py   一键启动和设备自检入口
    其他脚本           构建、离线测试及专项诊断
  config/              工作区级配置与升降限位观测记录
  env.sh               加载本机 ROS 2 和 SDK 环境
  navigation/          雷达、建图和导航适配，详见 navigation/README.md
  lift-low-speed/      升降低速实验源码和测试
  lift-upward-damping-candidate/
                       上升阻尼实验源码和补丁
  repos/               本机独立检出的厂家 SDK
  workspace/           本机 ROS 2 依赖及构建环境
  .venv/               本机 Python 虚拟环境

Git 保存项目源码、配置、测试和适配补丁。厂家 SDK、第三方依赖、编译产物、
日志和桌面快捷方式都保存在本机。Git 检出后，需要先准备对应的本机依赖和构建
产物，才能运行启动器。现有 ROG 已配置 Ubuntu 24.04、ROS 2 Jazzy、Python
虚拟环境、驱动二进制和 USB 转 CAN 设备映射。启动器调用 sudo -n 管理 CAN，
桌面弹窗使用 Zenity 和 GTK 3。

ROG 开机后如何启动
------------------

1. 登录 ROG 的 arx 桌面。确认机器人供电、急停已释放、双臂与升降周围已清空，
   并且现场人员能操作硬件急停。双臂驱动启动时可能使能或复位，机体驱动启动时
   会执行升降回零。系统重启后需要手动打开启动器。

2. 双击桌面「ARX 一键启动与自检」。也可在 ROG 图形桌面的终端执行同一入口。

       cd /home/arx/arx-competition-control
       /usr/bin/python3 scripts/start-console.py --desktop

3. 等待「ARX 设备自检」窗口。机械臂、底盘、升降各有一行，显示「好」或
   「不好」。故障原因写在对应行下，机械臂会注明左臂或右臂。「好」表示该项
   有实时反馈且网页控制接口就绪。已有正常运行的进程会被复用，关掉自检窗口
   后服务继续运行。

4. 三项就绪后打开控制台。ROG 本机用 http://127.0.0.1:9983/，同一受信任
   局域网的电脑用 http://192.168.58.14:9983/。如果 ROG 的 IP 变了，在
   ROG 上运行 hostname -I 查看当前局域网地址。

启动器依次确认 9983 网页服务、建立或复用 CAN 接口、启动缺失的驱动，再检查
ROS 2 实时反馈和控制接口。左臂用 can1，右臂用 can3，底盘和升降共用机体驱动
与 can5。网页与驱动使用 ROS 域 191。导航使用独立的 ROS 域 192，由导航
工作区单独启动。

自检异常时怎么查
----------------

- 网页能打开但读数是「—」时，先打开一键启动与自检。web/start.sh 只启动
  网页，完整链路还需要 CAN 接口和对应驱动。
- 某一项显示「不好」时，先看该行的故障提示。启动器新拉起进程的日志在
  web/hardware/runtime/launcher-*.log。当前已运行的服务也可能使用同目录下
  的 web.log 或带时间戳的驱动日志。
- 以下命令可只读核对网页反馈与 Linux CAN 接口。

       curl -fsS http://127.0.0.1:9983/api/state | python3 -m json.tool
       ip -brief link show type can
       ls -l /dev/arxcan1 /dev/arxcan3 /dev/arxcan5

  /dev/arxcan* 是 USB 转 CAN 设备，can1、can3、can5 是 Linux 网络接口。
  驱动运行时避免并行运行厂家控制程序或串口直连诊断脚本，以免争用设备。

历史调试记录
------------

下面保留初期离线搭建、标定和硬件验证记录。里面的 PID、设备状态和阶段性限制
属于记录当时的情况，日常启动和状态判断以上面的流程为准。

READ-ONLY REAL DEVICE VERIFICATION - historical result

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
