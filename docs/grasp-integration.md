# 识别、位姿与抓取路径预览

D405 软件预览在 `codex/grasp-integration` 的独立工作树完成验证，随后经用户明确授权发布到 main 工作区和运行中的 9983，仅重载网页进程。没有发送机器人运动指令，也没有启动或重启机器人驱动。

## 入口与操作

集成版本加载后，9983 的 D405 卡片新增「自动抓取链路预览」，对应 `/grasp-integration.html`。点击「识别并生成预览」才开始采集和计算；打开页面本身不会运行模型。

1. 现有 9983 采集程序临时申请 D405 深度，取得同一时刻的彩色、深度、右臂反馈和升降反馈，采集结束释放深度请求。
2. 已有电池识别程序运行 DINO、Qwen 复核和 SAM2 分割。识别不确定时返回原因并停止，不把失败结果继续用于正式位姿预览。
3. FoundationPose 使用同一张 RGB-D、分割掩码和电池 CAD 估计位姿。
4. 将 CAD 上的 F（提手前沿）、G（夹持参考点）、T（夹指伸入末端）转换到整车 base_link。
5. 在同一份实测点云上显示 G 和预抓点，生成 Y、Z 固定、沿 +X 的参考线段。预抓距离是从 G 向后退的距离，默认 50 mm；不是距机器人当前位置的固定前伸长度。

页面显示识别图、CAD 位姿图、可旋转点云、参考点坐标、有效深度比例和朝向偏差。点云坐标以米供 Three.js 绘制；页面坐标和线段用毫米。点云是实测表面，FoundationPose 提供对象位姿，CAD 定义夹持位置，几何计算负责生成线段。

「取消本次计算」复用 Agent 取消标志；模型子进程会终止并释放显存。采集阶段由既有采集接口的有限超时收尾。识别与位姿估计串行运行。整合入口内部有互斥锁；原有独立视觉、FoundationPose 页面不共享该锁，验证期间不要同时运行这些模型页面。

## Agent / tools 如何复用

复用现有 `AgentRuntime`、`ToolRegistry`、`RunStore`、网页会话、请求去重和工具回执；使用 competition-tools 分支已有的注册表扩展方式，没有另写一套 Agent 框架。

- `capture_grasp_scene`：经 `/api/handle-preview` 冻结 RGB-D，校验源快照 ID、外参哈希及图像/反馈时间。
- `preview_battery_grasp`：对指定 observation_id 串联识别、位姿、点云和路径。
- 保留状态与观测读取工具；这个注册表没有执行抓取、关节、底盘或升降运动工具。

网页按钮通过同一 Agent 任务机制执行固定两步工具流程，不需要语言模型决定是否执行运动。已有语言模型运行器也可使用这个受限注册表。本次未验证新的自然语言对话行为。

每次运行的源图、scene.npz、state.json、工具回执和预览结果保存在当前工作树 `agent/runs/`，不加入 Git。多个页面的结果按会话隔离。源文件哈希或快照 ID 不匹配时拒绝计算；所有后续模型使用归档的同一份 RGB-D，不再读取正在变化的实时图像。

## TCP 等待标定

当前 `tcp_status=pending`，腕部预抓点、腕部夹持终点都为 null，`executable=false`。旧的人工示教偏移不会代替新的 TCP。

TCP 完成后，需要确认标定点就是此次夹持使用的工具参考点，并提供它相对于右腕 `right_link26` 的位置，以及工具轴方向的定义。只有位置的枢轴标定不能单独确定工具坐标轴朝向。随后把目标工具位姿换算为腕部位姿：
`T_base_wrist = T_base_tool × inverse(T_wrist_tool)`。

J6 −90° 是约定夹法，不等于所有姿态下的整车 Roll 固定为 −90°。接入 TCP 后还需要对目标朝向、IK、整段夹爪扫掠碰撞和执行结果分别验证。这里输出的是接触参考点线段，尚不是可执行腕部轨迹。

本阶段未加入 D435 粗定位、底盘对齐、升降调整和闭爪执行；这些运动阶段不因 TCP 文件填好而自动启用。既有手眼数据、示教数据和 TCP 标定工作区均未改动。

## ROG 依赖

代码使用现有安装：
- 识别：`/home/arx/arx-vision-demo/vision/demo`
- FoundationPose 环境和模型：`/home/arx/arx-foundationpose/foundationpose`
- 手眼外参：本仓库 `vision/handeye/results/d405-right-cc200-ee0ff2fa49e9e56285d271d6.json`
- CAD：本仓库 `web/cad/gripper-v2/battery-print.stl` 和 `web/public/cad/battery-grip.json`

采集 API 和 RGB-D 缓存必须属于运行该代码的同一工作树。新工作树需使用已有模型、Three.js 与 RealSense Python 依赖；这些依赖和权重不随本次代码提交。不运行厂家示例来做软件测试。

## 软件验证记录

- Agent 测试：39 项通过，包含帧身份、防篡改、失配坐标、识别拒绝后不运行位姿、工具权限、取消/超时。
- FoundationPose 输入与 CAD 测试：8 项通过。
- 网页回归：209 项通过（新工作树补齐本机已有依赖后）；新接口验证会话与来源限制，假硬件调用为零。
- 浏览器离线回放：位姿叠加、点云、TCP 待标定、取消、失败后清除旧结果和输入范围校验通过，无页面脚本异常。浏览器回放使用诊断数据，不是实机自动抓取成功记录。

真实历史 RGB-D 的识别验证：约 202.26 秒、峰值约 1927 MiB，复核认为提手证据不足而拒绝。本次正式链路正确停止，没有得到可接受的端到端实景抓取结果。

同一历史帧另做诊断性后半段测试（不作为识别通过的结果）：FoundationPose 配准约 2.84 秒、峰值约 2751 MiB；位姿、点云和水平线段接口正常。结束后 GPU 使用回到 15 MiB。配准时间不包含识别、加载和采集；不能视为整条链路总耗时。

复现软件测试：
```sh
cd /home/arx/arx-grasp-integration
/home/arx/arx-competition-control/.venv/bin/python -m unittest discover -s agent/tests
/home/arx/arx-competition-control/.venv/bin/python -m unittest discover -s web/tests
/home/arx/arx-foundationpose/foundationpose/.venv/bin/python -m unittest discover -s foundationpose -p 'test_*.py'
node --input-type=module --check < web/public/js/grasp-integration.js
git diff --check
```
