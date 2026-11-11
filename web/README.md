# ARX 比赛网页控制台
## 2026-11-05 底盘平滑换向与零速制动

用户现场授权后已切换机体驱动和网页后端，升降回零状态和控制接口已就绪。
chassis_motion.hpp 的新逻辑已生效。用户已现场试用制动和平滑换向，反馈有阻尼感。
后续 Q/E 加速率已提高到原来的 4 倍并部署，尚待新一轮手感验收。

运动指令先经驱动级速度斜坡。SDK 内部单位的平移加/减速度为 0.8/1.6 每秒，
转向为 1.6/0.8 每秒。折算到网页原始命令为平移 2/4 每秒、转向 4/2 每秒。
Q/E 从静止到最大转向目标的斜坡时间约 0.25 秒（原 1 秒）。
平移合速度上限 2、转向上限 1 沿用现有配置，实际物理速度和加速度仍需标定。
任一轴换向时，三个分量先共同减到零，保持零速 0.2 秒，再向最新目标加速。

Space、松键和正常失联停车会立刻把速度目标置零，在厂家整体速度模式 1 下保持
0.8 秒，然后进入保护模式 2。期间新运动目标不会绕过制动窗口。这里请求厂家
速度环主动跟踪零速，实际制动力、停车距离和驻坡能力须现场验证。非法指令、
失去回零状态仍立即走保护路径。驱动在 250ms 指令超时后启动上述有界停车流程。

显式 chassis.stop 在网页没有运动归属时也会下发，避免先松 W 再按 Space 成为空操作。
轮速历史记录存在断流、缓存和单位不确定问题，本版没有加入依赖该反馈的外层 PID。
测试覆盖合成限速、加减速、换向零速停留、Space 零目标、重复停车、
制动窗口内的运动请求、250ms 超时及未回零保护。



## 2026-11-04 manual input fixes (current)

Reference inspected read-only on TH Ubuntu 192.168.58.68:
apps/web/public/js/main.js, manual joint Send and independent gripper buttons.
ARX retains its own SDK, axes, signed gripper range and software joint limits.

Edited targets now remain visible after Send; real feedback updates the actual
readouts. Restore actual feedback or switch arm to discard drafts and follow.
Close writes the explicit numeric target 0 to the gripper input/slider and sends
it independently; 0 is never treated as an empty value. Joint commands preserve
the gripper goal, and gripper commands preserve an active arm goal.

Absolute joint goals are accepted anywhere inside existing software limits.
The driver advances its reference at at most 5.7 degrees/second per axis;
max_delta_deg=5.7 now bounds reference lead over actual feedback, rather than
rejecting an entire absolute goal farther than 5.7 degrees away. Software Stop
cancels the pending goal and holds measured positions. This rate/lead bound is
a commissioning choice, not a manufacturer maximum-speed specification.

Each request carries kind plus UUID in RobotCmd.header.frame_id; the driver
returns an acceptance/rejection on /web/<side>/command_result. HTTP success
requires driver acceptance; acceptance does not prove actual arrival. Waiting
for acknowledgement releases the controller lock so ROS feedback can continue.

Validation: 38 backend tests, C++ target/slew/limit checks, 7 frontend state tests;
real Edge browser checked close-zero, retained targets, bounds and restore,
with all POST requests intercepted and no browser-test hardware commands.
Live acknowledgement/current-angle/close-zero checks: hardware/runtime/input-fix-live-validation.json.
Small right-gripper -0.3 -> 0 check: hardware/runtime/gripper-zero-check.json.
Actual feedback moved approximately -0.30 -> -0.084 raw; precise zero or grip force
is not claimed. No large-angle or full-range movement test was performed.


## 2026-11-03 X5 2025 real arm connection

Both arms now publish manufacturer joint coordinates and gripper motor position at 100 Hz:
left /arm_l_status_full on can1, right /arm_r_status_full on can3, ROS_DOMAIN_ID=191.
The existing page follows actual values while unedited; edits stay pending until Send.
Switching arms restores that arm's actual values. No raw motor angles are relabeled as joint angles.

hardware/x5_controller_web.cpp wraps the ROS2 InterfacesThread SDK (end type 2,
x5_2025.urdf). start-arm.sh pins the ROS2 libarx_x5_src.so before the Python SDK
library directory: their identical filenames do not imply compatible APIs.
Only /web/left/command and /web/right/command are subscribed; no shared Joy/VR inputs.
Both drivers hold measured arm/gripper positions after receiving valid feedback.
SDK construction enables motors and may move/reset them; startup requires onsite clearance.

Joint and gripper commands use mode 5, with manufacturer/software ranges and the
existing 5.7 degree per-command joint deviation checked again by the driver.
Mode buttons 0..3 use the documented vendor state enum. Actual mode is not echoed,
and destructive/free-motion modes were not physically exercised for verification.
TCP feedback is visible; TCP motion remains disabled because its safe workspace is unset.
Stop uses /web/<side>/stop Trigger, captures measured arm AND gripper positions,
and enters position hold; it is a software stop, not a hardware emergency stop.

A receive-only SocketCAN monitor checks replies from all seven motors, excludes
host-generated loopback using MSG_DONTROUTE, and withholds feedback after 300 ms
without all replies. Motor errors/lost feedback latch damping and driver not-ready;
restart/recommission with onsite monitoring before recovery. Never run the serial
register-query scripts while these SDK drivers own the adapters.

Current live verification: hardware/runtime/arm-live-connection.json.
33 backend tests, 7 browser-state tests, C++ arm/body guard checks passed.
Both arm ready topics and measured feedback are present. Finite 1-degree J6 and
0.05-raw gripper tests went through the web HTTP API, including stop and return
requests. Feedback changed, but precise target tracking, full joint travel,
loaded operation and mode 0..3 physical effects were not accepted as verified.
No navigation or body-driver restart was performed for this arm connection.
No automatic boot start was installed.


位置：/home/arx/arx-competition-control/web

TH 9983 只作为配色、控件、输入校验和目标编辑/实时反馈交互参考。
没有复制 TH 的 SDK、CAN、语音、3D 模型或机器人服务。
后端为 Python 标准库 HTTP 服务，直接通过 rclpy 接入本工作区的 ROS 消息；
没有新增第三方依赖。

## 当前机体真实接入状态（2026-11-03）

用户已授权并确认现场就绪，当前仅机体真实执行开启。机械臂 verified=false，can1/can3 未启动。
机体唯一驱动由 web/hardware/start-body.sh 启动，复用比赛工作区 ROS2 厂家库，固定库路径并检查符号。
底盘只订阅 /web/body/command，不接收共享 VR、Joy 或机械臂话题，不改变升降/头/腰目标。
驱动以 steady_clock 独立检查 250ms 断流停车；网页另保留 300ms 租约。
网页平移/转向输入现在是原始 PosCmd 数值，不声称 m/s、rad/s：平移 0 < 值 <= 2.0，转向 0 < 值 <= 1.0。
网页输入框可自行设定当前值；编辑后重新开启运动模式生效，刷新后恢复默认 2.0 / 1.0。
底层驱动同步限制为缩放 /2.5 后的 0.8 / 0.4；上限是用户指定的软件限制，不是厂家额定速度。
正负方向由键位决定；适配器只翻转 Y，机体内按厂家 /2.5 缩放。配置键 max_linear_m_s / max_angular_rad_s
因兼容保留旧名称，此接入版本数值含义为原始 SDK 输入；不能乘时间计算厘米。
厂家库保护模式会保留上一笔轮速，ROS 消息新鲜不等于轮速缓存已刷新。网页已标注。
现场仅确认原始前进1.0、0.2秒有可见前移并停稳，其他方向与实际距离未作物理验收。
升降目标0..18rad，目标斜坡0.5rad/s；每次驱动启动会重新回零，反馈不换算物理高度。
/web/body/calibrated 必须持续为true才开放网页机体控制。
停止升降走 /web/body/stop_lift Trigger，取消剩余目标、保持厂家库当前 ramp 参考，
不再将带稳态偏差的实测位置重设为目标。小幅运动和重复停止检查已通过。
固定高度参数服务仍为 /lift_controller/set_parameters。实际存在跟踪偏差，不代表目标精确到位。
此接入未用实验性升降阻尼；未修改厂家仓库、旧工程和已有导航工程。没有开机自启。
接入记录见 hardware/runtime/live-connection.json；旧配置备份见 config-before-enable.json。
以下早期“观察模式”段落为实现初期说明，当前执行配置以本段及 config/control.json 为准。

## 目录

- src/server.py：静态网页、JSON 指令接口、SSE 实时反馈。
- src/control.py：输入/范围/反馈校验、控制分发、底盘指令租约。
- src/ros_adapter.py：ROS 观察节点和受控指令适配，不创建 SDK 对象。
- public/：页面、CSS、浏览器逻辑。
- config/control.json：独立控制接口与校验配置。
- tests/：纯软件测试，用 FakeAdapter，不接电机。

## 启动

    bash /home/arx/arx-competition-control/web/start.sh

默认地址 http://192.168.58.14:9983；主机可用 http://localhost:9983。
WEB_HOST、WEB_PORT 可覆盖监听地址和端口。WEB_ROS_DOMAIN_ID 默认为 191，
用于读取现有隔离域的反馈；不修改其他驱动的 ROS_DOMAIN_ID。
没有配置开机自启。

仅在受信任局域网使用。服务器检查浏览器同源请求并使用 Strict 会话 cookie；
这不是向不受信任网络开放控制端口的身份认证方案。

## 当前运行边界

execution_enabled 默认为 false。页面接收真实 ROS 反馈，但所有执行请求
在后端拒绝，包括各部件停止请求。打开网页、读取状态和建立事件流不产生
运动指令，不启动/重启驱动、不归零、不调用 SDK 构造函数。

ROS 订阅：
- /arm_l_status_full、/arm_r_status_full：arx5_arm_msg/RobotStatus。
- /body_information：arm_control/PosCmd。

没有有效反馈时显示不可用；历史文件和原始电机采样不冒充实时关节坐标。
夹爪先显示 SDK 原值，未标定前不显示虚假的百分比/毫米。
升降显示 SDK 反馈，不声称是校准物理高度。

## 实机控制接入前

需要另行现场验证和授权，然后填写本机配置。不能只改 execution_enabled：
所有路径还检查反馈新鲜度、验证标记、范围和订阅者/服务。

机械臂适配厂商 normal 模式：
- 指令：/web/left/command、/web/right/command（RobotCmd）。
- 反馈：/arm_l_status_full、/arm_r_status_full。
- 模式 5 为关节控制，模式 4 为末端控制，来自本工作区 InterfacesThread::state。
- 网页角度为度，ROS 弧度；网页位置毫米，ROS 米。
- 需要六轴限位、夹爪范围/开闭端点、单次偏差和 TCP 可达范围。
- 停止语义为保持已测关节、保留夹爪，需实机验证。
- 不使用与机体共享的 /ARX_VR_L，防止联动。

底盘通过独立 /web/body/command（PosCmd），需要机体驱动把 VR 输入重映射
到该话题。适配 LIFT robot_type=0 的 2.5 缩放及 Y 反号，
同时保留高度、腰部和头部字段。方向、单位、停车均需现场验收。
网页不会更改现有 /calibration_isolated/*，不会接管现有驱动。
底盘开启网页运动模式后，按住键盘方向键连续发送，松开/失焦/隐藏/断线请求停车；
后端 0.3s 租约和递增序号处理断连与乱序，停止之后的迟到旧指令会被拒绝。
驱动级超时停车仍需独立验收，后端崩溃时不能依赖网页来保证停车。

升降适配现有低速程序 /lift_controller/set_parameters 的 fixed_body_height。
0..18.0 是软件候选范围，不是已验证全局物理限位。
必须确认本次归零参考、反馈符号/圈数及保持行为。
启动网页不会调用参数服务；只在控制入口启用且检查通过后才发送目标。
参数接口应只有唯一目标控制器。

观察模式启动和软件测试不构成实机运动验证。

## 软件验证

    cd /home/arx/arx-competition-control/web
    ../.venv/bin/python -m unittest discover -s tests -p 'test_*.py' -v
    node --test tests/state.test.mjs

测试不运行厂商 demo，不启动硬件驱动，不发送运动消息。

## 方舟无限限位依据

六轴、夹爪和升降目标限制已按厂家资料与用户指定范围配置。
详见 LIMITS.md 和 config/limit-sources.json；不使用 TH 机械臂限值。
未知底盘速度、TCP 安全工作区保持 null，真实控制与实机验证标记仍关闭。

## 方舟无限机械臂模式按钮 0 / 1 / 2 / 3

页面模式区跟随顶部左右臂选择；只发送到所选机械臂的专用 RobotCmd 话题。
4/5 的自动选择及现有软件停止逻辑保持原样。

| mode | 按钮 | 说明 |
| --- | --- | --- |
| 0 | 力矩清零 | 不主动输出托举力矩，可能下落；不是保持或急停。 |
| 1 | 机械臂复位 | 主动运动到驱动 go_home_position 初始姿态；不是标定零点。 |
| 2 | 阻尼模式 | 对运动增加阻力，不能当作位置锁定。 |
| 3 | 重力补偿 | 补偿机械臂自重，便于拖动；附加末端负载影响补偿。 |

接口沿用 POST /api/command：
`{"type":"arm.mode","arm":"left","mode":2}`。
mode 必须为整数 0..3，拒绝字符串、浮点、布尔值、4/5 和额外目标字段。
后端读取所选臂新鲜实测 joints / tcp / gripper，构建完整 RobotCmd，再复用 ROS adapter。
这样不把 JSON 中缺省字段的零值当成关节、末端或夹爪目标；原样携带夹爪反馈不等于已经实机证明切换时夹持力不变。

安全检查继续使用 execution_enabled、机械臂 verified、反馈有效期、配置范围和专用订阅者。
每臂新增 verified_modes: []，只有逐个完成现场验证的模式才可加入该臂列表。
当前列表为空，真实控制仍关闭；只改全局开关不能启用这些按钮。
模式 1 的实际复位目标及路径由驱动负责，网页单次关节差值限制不约束它的内部复位轨迹，
必须核对 go_home_position、限位、路径和速度后才能把 1 加入 verified_modes。
0/2/3 的支撑、阻尼及负载效果也需要分别确认；不得当作软件停止替代。

RobotStatus 不包含当前 mode，因此不显示选中高亮或“切换成功”。
接口只返回已发送请求，日志明确记录所选臂、模式编号和“实际模式未确认”。
页面的待发送滑块草稿不会因请求模式而自动发送或清空，左右臂切换不发送模式请求。

来源：
- 本工作区 repos/ARX_X5/.../interfaces/InterfacesThread.hpp 的 state 枚举：
  SOFT=0, GO_HOME=1, PROTECT=2, G_COMPENSATION=3, END_CONTROL=4, POSITION_CONTROL=5。
- X5Controller.cpp 的控制消息回调 将 RobotCmd 位姿、关节、mode 和 gripper 传给 SDK。
- 厂家 ROS2 手册模式表将 2 描述为阻尼模式；不要把它与底盘 PosCmd.mode1 的保护编号混用。

实现与验证只涉及 web 文件夹，不启动或重启硬件驱动，不发送真实模式命令。

## 底盘键盘运动模式

网页移除鼠标方向盘，键位提示使用不可点击的 kbd 元素。独立“刹车”按钮保留。
点击“运动模式”后本页才捕获键盘；W/S 前进/后退，A/D 左移/右移，Q/E 左转/右转，
Space 请求软件停车。按下时高亮相应键位，提示的是输入/请求，不声称车已开始移动或物理刹停。

仅在网页连接、真实控制、底盘控制及停止能力均可用时才能开启。
开关状态不保存、不随重连自动恢复；页面打开或切换模式只改变键盘监听，不发送运动目标。
方向键按住每 100ms 用一条 chassis 指令发送 x、y、omega，松开最后一个键发送 chassis.stop。
后端 0.3s 租约、控制归属、指令序号及速度检查保持原样。

不同轴可以组合，例如 W+A+Q 同时前进、左移和左转。斜移按平移合速度限幅，
转向沿用独立限速。相反方向同时按下会请求停车，并等待全部松开。
Space 优先于方向键；释放 Space 时仍按着的方向键不会恢复运动，必须全部松开再重新按下。
按键重复事件不会启动额外定时器，也不会在重新开启模式时恢复仍按着的旧键。
退出运动模式、窗口失焦、页面隐藏/关闭、焦点进入输入框、修饰键/输入法组合、
反馈连接断开、底盘能力失效或运动请求失败都会清除按键并请求停车。
编辑输入框不截获 WASD/QE/Space，避免输入数值或文本时误动。

这里的“运动模式”只是本网页键盘控制开关，不改变厂家底盘模式，也不启用真实执行。
Space 和网页刹车按钮都是软件请求，不代替硬件急停或驱动级断流停车。

## 日志查看及反馈诊断

顶部“日志”按钮打开底部日志面板，交互参考 TH 9983；关闭按钮或 Escape 收起。
网页事件和后端事件分开标注，带时间戳与警告颜色，最多显示最近 100 条。
可取消“自动滚动”后阅读历史。日志保存在当前进程/页面内存，不是持久日志归档，
也不代表已接入所有外部驱动 stdout、原始 CAN 帧或每帧关节数据。
打开日志会退出本页底盘键盘运动模式；不会启动硬件或发运动目标。

后端观察节点每秒只读检查配置话题的 ROS 发布者、消息类型及反馈有效期，
只在初始状态、状态或发布者数量变化时记录日志，不连续刷每帧反馈。
能区分：无发布者、有发布者但尚无有效反馈、类型不匹配、反馈有效、反馈过期和图查询失败。
ROS 图查询暂时失败不会停止反馈订阅。关节区域同时显示所选臂的具体反馈原因。
日志中“已下发”或“已请求”不代表实机运动或模式已确认。

2026-11-03 本次只读检查：Domain 191 只有 lift_controller 和网页观察节点，
/body_information 有 1 个发布者；左右臂 _full 反馈话题发布者均为 0。
Domain 0 未发现机械臂驱动；内核 CAN 网络接口只有 can5，can1/can3 未启动，
但 /dev/arxcan1、/dev/arxcan3 USB 设备链接存在。
因此当前机械臂无实时反馈的直接原因是没有运行反馈发布链路，
不应归因于网页观察模式禁用了订阅。启动厂商控制器可能使能/复位，未在本次工作中执行。

后续 normal 驱动必须显式将 arm_pub_topic_name 配成网页订阅的 /arm_l_status_full 和
/arm_r_status_full（或两端一致修改）；normal 原样使用参数名发布，只有 vr_slave 自动加 _full。
本次没有猜测切换话题或自动发现并接管驱动。

## TCP 定点标定采集

控制台的机械臂区域提供 `tcp-calibration.html` 入口。页面只读取左右臂关节、升降和
`base_link -> J6 wrist` 正运动学，不调用 `Controller.submit()`，因此记录、求解和保存候选都不会
发送机械臂、升降或底盘命令。

操作者把最终工具尖端保持在同一个固定点，手动改变腕部方向并逐点确认。每臂至少 6 个样本，
建议采集 8～12 个方向。后端用线性最小二乘求解 `R_i p + t_i = c`，返回腕部坐标系中的
TCP 平移、固定点位置、秩、条件数、RMS、最大残差和逐点残差。姿态变化不足或反馈超过
0.3 秒会拒绝记录或求解。

结果只包含 XYZ 平移，不包含工具坐标系方向。点击保存后写入
`web/hardware/runtime/tcp-calibration/candidates/`；它是待验证候选，不会自动写入控制配置，
也不会启用厂家 `arm.tcp` 通道。枢轴算法的选型与交叉核对参考
SciKit-Surgery `scikit-surgerycalibration/algorithms/pivot.py` 的 BSD-3-Clause 实现；本项目使用
现有 NumPy 依赖重新实现最小线性求解，没有引入该项目的完整依赖树。
