# Agent 自动抓取链路：当前实现与验证边界

当前开发工作树 `/home/arx/arx-grasp-integration`，分支 `codex/grasp-integration`。本轮新增代码仅做软件测试，尚未加载到运行中的 9983，没有发运动指令或重启驱动；已离线复算一张历史 D405 快照。完整目标仍是 D435 发现电池 → 升降和右臂对位 → D405 精定位 → 水平前伸 → 闭合并验证持物；当前实现尚未完成执行部分。

## 页面与阶段工具

加载集成版本后，9983 的「自动抓取链路预览」入口对应 `/grasp-integration.html`。点击「从 D435 开始预览」后使用真实 `AgentRuntime` 模型—工具循环，已移除旧 `/preview` 固定两步调用。打开页面本身不会启动模型。

1. `get_system_status`：查询已有控制台的反馈和能力。
2. `observe_battery(camera=d435)`：经现有采集进程临时申请深度，保存配对 RGB-D 和反馈；已有 DINO、Qwen 复核和 SAM2 分割识别电池，估计可见表面在 base_link 的粗位置。使用 D435 自己的 CAD 外参，软件深度窗口最大 3 m 不代表此范围内的精度保证。
3. `plan_coarse_alignment(observation_id)`：依据上述定位、CAD 零关节腕位和现有 41.54 rad/m 名义比例提出升降建议。目标是让参考腕位低于目标表面约 100 mm，随后由机械臂向上够。目标 Z 先扣除 CAD 腕部零位高度再换算升降行程；遵守 control.json 升降范围，显示限幅后的高度残差。零关节姿态只是计算参考，不会回零。
4. `observe_battery(camera=d405)`：检查当前右腕视野并保存新的配对 RGB-D，使用既有小板手眼外参。当前没有执行粗调，所以这仍是现姿态观察，不能称为完成了相机交接。
5. `preview_battery_grasp(observation_id, clearance_mm)`：复用同一 D405 快照的识别掩膜，运行 FoundationPose，把 CAD 的 F（提手前沿）、G（夹持参考点）、T（夹指伸入末端）转换到 base_link，再显示实测点云与 Y/Z 固定、沿 +X 的接触点线段。预抓距离默认 50 mm，是 G 向后退的距离，不是从当前腕位固定前伸 50 mm。

按业务阶段封装工具，点云变换、CAD 点换算和 GPU 生命周期仍是内部函数。复用已有 `ToolRegistry`、`RunStore`、云端配置、调用预算、工具回执、请求去重、取消和网页会话隔离。专用抓取提示词不再包含早期“只有 RGB、相机对应未知”的描述。

工具代码强制证据顺序：头部定位未成功不能跳到精抓；每台相机每轮最多观察两次。重新采集开始即清除旧预览；新头部观测还使旧粗规划和腕部观测失效。模型文字不能把任务标成抓取成功。

页面分别显示 D435 结果、升降建议、D405 结果和位姿/点云。未发现电池、识别拒绝和深度不足分别报告；D405 不可见时保留 D435 结果。单相机内的后续计算使用归档快照；两台相机不是同一曝光时刻，也尚未验证跨相机是否为同一目标。

## TCP 与运动边界

`tcp_status=pending`，TCP、腕部预抓点和夹持终点为空，`executable=false`。按用户要求等待新的 TCP，旧示教偏移不替用。CAD 固定尺寸照 CAD 使用，实时反馈保留实测值，不四舍五入伪装标定常量。

TCP 需说明标定点相对于右腕 `right_link26` 的位置、工具轴方向及其与 CAD 夹持参考点的关系。只有位置的枢轴标定不能独自给出工具轴方向。目标换算关系是 `T_base_wrist = T_base_tool × inverse(T_wrist_tool)`。J6 约 -90 度是约定夹法，不等于所有姿态的整车 Roll 都为 -90 度。

尚待完成：升降时双臂/夹爪/环境扫掠检查；可达且可见目标的腕部观察姿态；执行粗调及确认同一电池；跨视角位姿一致性与实物精度验证；TCP 轨迹换算；接近后 D405 重测纠偏；连续 IK 和全路径碰撞检查；命令所有权、反馈控制和取消；到位后闭合及动作后真实持物验证。当前没有硬件执行工具，填入 TCP 也不会自动放开这些阶段。没有导航和锁止电池强行抽取功能。

## 数据、依赖与资源

每轮图像、scene.npz、工具回执、模型产物和计划保存在该工作树 `agent/runs/`，不入 Git。检查源快照 ID、外参哈希和图像/反馈时序；D435 外参固定于车体，D405 使用拍照时的右腕 FK 和升降反馈。深度采集沿用现有唯一相机进程，结束归还租约并恢复此前设置。

识别依赖 `/home/arx/arx-vision-demo/vision/demo`；FoundationPose 环境依赖 `/home/arx/arx-foundationpose/foundationpose`。本仓库保留小板手眼外参、D435 CAD 外参、电池 STL 和 `web/public/cad/battery-grip.json`。不下载新权重、不运行厂家运动示例。

「取消」停止 Agent 后续调用并终止模型子进程；采集由已有有限超时收尾，不能把 Agent 取消当硬件急停。识别和位姿串行，整合入口共享 GPU 锁；独立视觉/位姿诊断页面不共享该锁，避免同时运行。每个模型子进程退出后释放其显存。

## 软件验证

2026-11-12：Agent 51 项、网页 216 项通过；JS 语法检查和 git diff --check 通过。脚本化模型测试覆盖 D435 → 粗规划 → D405 → 位姿/路径调度、回执去重、识别结果复用、拒绝后不得越阶段、有限重试、失败后废弃旧路径、空 TCP 和 HTTP 会话/来源限制。假硬件调用为零。

本轮未做云模型现场调用、浏览器视觉验收和真实运动，不能据此称完整自动抓取已跑通。早期 D405 历史帧诊断曾通过后半段点云与位姿显示，但识别复核拒绝，不能算识别成功的端到端证据。

```sh
cd /home/arx/arx-grasp-integration
/home/arx/arx-competition-control/.venv/bin/python -m unittest discover -s agent/tests
/home/arx/arx-competition-control/.venv/bin/python -m unittest discover -s web/tests
node --check web/public/js/grasp-integration.js
git diff --check
```

## CAD 渲染深度一致性检查（2026-11-12）

FoundationPose 在与输入一致的去畸变相机下渲染 CAD 深度，与同一快照的实测轴向深度比较：识别掩膜内有效深度比例、CAD 轮廓覆盖、渲染区域命中掩膜比例、重叠区域深度绝对差中位数和 P90。孔洞和无效深度不按零误差处理。深度不足、明显错位或旧结果缺少质量回执时，保留诊断图但不生成接触点路径；Agent 不能用文字绕过该检查。

当前软件初始筛查阈值写在 `foundationpose/pose_quality.py`：至少 100 个配对像素、掩膜深度有效率至少 50%、轮廓覆盖至少 70%、渲染精确率至少 60%、深度绝对差中位数不超过 10 mm、P90 不超过 25 mm。这些是初始筛查策略，尚未通过有真值的现场样本标定；通过不等于 TCP 精度、正确提手朝向或碰撞安全。遮挡、黑色材料深度噪声也可能导致拒绝。

离线复算历史冻结快照 `239e9263c1ec4d34b84ac5d00a4ebf46`（使用历史 ROI 和 GrabCut，不是本轮 DINO 复核结果）：11444 个配对像素，有效深度 95.59%，轮廓覆盖 99.34%，渲染精确率 86.05%，深度中位绝对差 4.94 mm、P90 26.41 mm。按当前策略因尾部误差超过 25 mm 被拒绝；这不能单独证明外参错误，也不能当作抓取误差测量。模型注册 2.85 秒，总计算 8.62 秒，PyTorch 峰值分配显存 2751 MiB（非整机显存总量）。结果位于本工作树 `foundationpose/runtime/depth-fit-check/`，不入 Git，未接相机或机器人驱动。

本阶段测试：Agent 57 项、网页 216 项通过；JS 语法及 diff 检查通过。涵盖零深度不可冒充准确、错位/遮挡轮廓、尾部误差拒绝、质量回执缺失不得生成线段，及通过后保留空 TCP。未部署。

## 前伸逆解计算内核（离线，TCP 仍空）

`agent/grasp_path.py::plan_insert` 接收冻结起始关节、升降、base_link 下预抓/夹持点、明确的工具朝向，以及以米为平移单位的 `T_wrist_from_tool`。TCP 为 None 时直接返回 tcp_pending，腕部终点和关节轨迹为空。当前页面仍不读取 TCP 或调用运动接口；这个计算内核供后续已验证的运动规划阶段使用。

给定明确工具变换时，先把接触点换算到腕部；接近预抓点单列为关节空间建议；随后每 2 mm 求一次连续局部逆解，保持工具 Y/Z 和朝向不变。拒绝不可达点、超过 5 度的局部关节跳变、中央立柱间隙不足；检查段间线性关节插值的 TCP 曲率误差上界，超过 1 mm 拒绝。姿态误差按最多 0.1 度关节间隔采样检查，超过 0.3 度拒绝，不能把该采样说成连续姿态证明。

输出始终 executable=false。尚无整机/夹爪环境碰撞、动态限速时序或真实控制回执。1 mm 是计算模型下的插值界限，不是标定精度或实际跟踪精度。测试使用真实交付 URDF 与合成 TCP（含非零旋转）做 10 mm 前伸复算，并测试未给 TCP、错误方向、不可达、中途逆解失败和立柱拒绝；合成 TCP 不会保存为真实标定。


## 升降沿途双臂立柱检查（离线）

粗对位工具现在读取冻结帧内配对的左右臂关节及升降反馈。采集复用现有反馈订阅，左臂需至少 4 个不同时间戳、覆盖 0.35 秒，变化小于 0.25 度，与图像相差不超过 0.2 秒，才随 NPZ 保存 `left_joints` / `left_arm_time`。左臂缺失不打断视觉观察，但旧帧或无效配对会使升降检查返回 unavailable，不能报双臂通过。

`agent/lift_sweep.py` 使用交付模型的保守包围盒，在双臂关节保持拍照时数值的假设下，每不超过 5 mm 高度检查一次升降全程，并扣除半个采样间距给出立柱间隙下界。上下行都检查，间隙需大于 20 mm。返回 column_clear / column_rejected / unavailable，页面和工具回执同时显示。

这仅检查中央立柱；周围物体、线缆、自碰撞及左右臂相互碰撞尚未覆盖。41.54 rad/m 仍是名义比例。`both_arms_sweep_checked=false`、`environment_checked=false`、`executable=false` 保留，不能把局部检查通过当作可执行许可。机器人未运动，驱动未重启，TCP 仍空。

本阶段软件验证：Agent 66 项、网页 217 项通过，JS 语法和 diff 检查通过；未部署或进行实机验证。


## 2026-11-12：视觉结果接入腕部插入规划（源码阶段）

`BatteryExecutionTools.plan_battery_pick(observation_id)` 在粗对位完成且重新取得
D405 观察后，读取本轮已通过复核的 FoundationPose／点云预览。它核对预览哈希、
D435/D405 同目标记录、深度归档、手眼和 CAD 参考版本，再调用已有 `plan_insert`。
新观察或重新计算预览会使上一份抓取计划失效，失败不能沿用旧结果。

当前没有配置实际 TCP，所以实际返回 `tcp_pending`，腕部目标与关节轨迹为空。
完整参数只允许本地控制程序通过 `BatteryExecutionTools(..., pick_setup=...)` 注入，
Agent 参数只有 `observation_id`，不能自行填写矩阵、容差或运动限速。
`pick_setup` 必须包含：

- `calibration_id`、`parent_frame=right_link26`、`tool_frame=grip_reference`、`translation_unit=m`。
- 完整刚性变换 `T_wrist_from_tool`，包括旋转；TCP 旋转不能从仅有平移的定点标定中猜出。
- `alignment_tolerance_deg`、`j6_tolerance_deg`，由实际夹指与电池的对位验证确定。
- `timing_limits`：六轴 `joint_speed_deg_s`、`joint_acceleration_deg_s2`，以及
  `tcp_speed_mm_s`、`tcp_acceleration_mm_s2`。测试中的参数仅用于软件验证。

夹持坐标定义：原点为 CAD 夹持参考 G，+X 沿伸入方向，+Z 背离朝下凸起，+Y 构成右手系。
只接受与整车 +X／凸起朝下相容的电池姿态；预抓和插入的夹持坐标旋转固定为整车轴向。
完整 IK 后还检查 J6 接近约定 -90°，不把 J6 单独当作世界姿态。

输出保留接近段、连续插入关节轨迹、时序和证据版本；即使 IK 通过也始终
`executable=false`。完整环境／双臂自碰撞、预抓后重新观察修正、现场标定与抓持核验
仍需完成后续连接。此次未注册到正在运行的 9983，未发送运动或重启驱动。


## 2026-11-12：预抓后有限纠偏判据

`agent/grasp_refinement.py` 比较已执行接近所用的旧计划与新 D405 结果，
用新帧的实际 TCP 计算 XYZ 偏差，返回已对齐、需要有限修正或需要重新定位。
目标、手眼/TCP/CAD/URDF/控制配置身份必须一致；新旧快照不同，插入方向及预抓距离不变。
升降变化、过大的目标跳变、夹姿旋转偏差不转化为追随运动。

纠偏阈值与最多次数由本地策略显式提供，次数最多 3 次，并累计修正距离；
实际参数仍待现场 TCP 与夹指容差确认，测试数值不作为默认控制参数。
本步骤仅有纯计算；后续由阶段工具强制到位后的重新采集时间及累计预算。
软件验证：6 项纯计算测试通过，覆盖 XYZ 修正、预算耗尽、版本/快照错配及非有限参数。

### Pregrasp measured arrival (software only)

ApproachTrajectoryStep reuses the existing trajectory ownership, driver receipt, fresh measured settling and failure-stop logic. Its arrival handler cannot close the gripper: ApproachExecutionBackend completes only with a mandatory new-observation receipt. No live registration or device restart. Four fake-transport approach tests and eighteen existing pick tests passed.
