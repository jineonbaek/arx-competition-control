# 美团决赛任务 Agent

Python 3.12+，仅用标准库。入口从**仓库根目录**运行。独立开发工作区为 `/home/arx/arx-agent-framework`，分支 `codex/qwen-agent-framework`。现有控制工作区和相机进程继续承担硬件控制与采集。

## 当前能做什么

- Qwen3.8-Flash 主 LLM：流式思考模式、非流式回复、函数调用、多轮工具结果回传、真实图片附件。
- 三相机只读采集：复用 9983 的 d435/d405/astra 原图或检测标注图，保存不可变图片和对应元数据。校验帧时间、来源文件名、序列号、JPEG尺寸、读取大小。
- 任务状态机：目标选择、抓取候选、规划、抓取、持物验证、收臂、导航、槽位选择、对准、插入、释放、稳定3秒验证、完成。
- 严格工具参数、调用ID去重、串行执行、预算、运行记录、取消、未决物理回执查询、恢复。
- 抓取/规划/控制/验证/导航/插入服务具有明确的 HTTP 契约。**当前均未接入，运动默认关闭**。配置了地址也不代表该服务通过实机验收。

当前相机接口不提供原始深度数组或机器人外参；D435 已有内参、部分表面测距及 YOLOE 框作为观测元数据保留，不能代替完整抓取几何。三路相机与机器人状态不保证同步。不能据此宣称 GraspNet、导航、插入或自动比赛已经可用。相机型号与头/胸/手位置对应仍需确认。

## 配置与密钥

非敏感配置见 `agent/config.example.json`。云端配置只引用环境变量。用户授权的密钥在 ROG 的 `agent/.secrets/qwen.env`，目录权限700、文件600，已被 Git 忽略。密钥不得写进配置JSON、命令行参数、测试或提交。支持新版带点号的完整密钥；按字面读取，不执行环境文件里的 shell 代码。

```text
DASHSCOPE_API_KEY=<完整密钥>
QWEN_BASE_URL=https://maas.qianwenaiapi.com/compatible-mode/v1
```

该地址来自[千问平台首次调用文档](https://platform.qianwenai.com/docs/developer-guides/getting-started/first-api-call)。模型产品页的示例曾指向另一套 DashScope 地址，不能混用不同平台的密钥与地址。[函数调用文档](https://platform.qianwenai.com/docs/developer-guides/tool-calling/function-calling)说明模型返回工具请求，实际工具由应用执行。

本地 profile 预留 `Qwen3.5-2B`、`http://127.0.0.1:8000/v1`。尚未下载模型、选择量化/推理框架或启动本地服务；实际模型别名和视觉支持须按将来服务配置。没有声称其工具调用质量或显存表现通过验证。

## 使用

```bash
cd /home/arx/arx-agent-framework
python3 -m agent --env-file agent/.secrets/qwen.env doctor
python3 -m agent tools
python3 -m agent observe --cameras d435 d405 astra
python3 -m agent observe --cameras d435 --view annotated
python3 -m agent --env-file agent/.secrets/qwen.env smoke
python3 -m agent --env-file agent/.secrets/qwen.env run --instruction '读取三路相机，描述看到了什么，不能移动机器人'
```

`doctor` 只检查配置与密钥是否存在，不回显密钥，也不承诺网络连通。`observe` 不调用模型。`smoke` 向模型提供状态/相机只读工具，测试“LLM→工具→JPEG→LLM”；确实采集到图像并正常返回才以退出码0结束。云端模型调用会传输所选图片并产生 API 用量。一般 `run` 退出码0表示正常结束对话，**比赛成功要看 `mission_success: true`**。

每次运行返回 `run_dir`。记录在忽略目录 `agent/runs/run-.../`：

| 文件 | 内容 |
| --- | --- |
| `state.json` | 任务阶段、调用回执、预算、消息历史、最终结果 |
| `events.jsonl` | 工具/模型轮次/错误事件，日志不输出密钥或模型思考内容 |
| `observations/obs_.../manifest.json` | 相机来源、时间、元数据、同步与几何可用性声明 |
| `observations/obs_.../*.jpg` | 本次观测的固定图片 |
| `writer.lock` | 单个运行的写入者身份；正常退出删除，异常退出由PID检查恢复 |
| `cancel.flag` | 请求停止 Agent 后续工作 |

运行目录权限700；协议历史可能包含用户文本、图像描述和模型返回的 reasoning 字段以支持后续协议，不应公开。历史只记录图片引用，发送请求时才构建 base64。

```bash
python3 -m agent status agent/runs/<run-id>
python3 -m agent cancel agent/runs/<run-id>
python3 -m agent --env-file agent/.secrets/qwen.env resume agent/runs/<run-id>
python3 -m agent reconcile agent/runs/<run-id>
```

恢复沿用原任务、调用ID和累计预算，不重新发送未决物理操作。取消标志不会自动清除；取消后的 `resume` 仍停止。`reconcile` 只查询现有回执。**取消 Agent 不等于硬件急停，也不能撤销已交给控制器的动作。** 本阶段没有发出机器人动作。

## 模块与任务边界

| 模块 | 职责 |
| --- | --- |
| `config.py`, `model.py` | 配置、凭证引用、Qwen协议、完整回复校验 |
| `observations.py`, `storage.py` | 只读相机快照、证据文件、原子状态保存、运行锁 |
| `tools.py`, `contracts.py` | 工具注册、JSON参数校验、前置条件、请求回执 |
| `mission.py` | 比赛状态转移、持物与放置证据规则 |
| `backends.py` | 与确定性领域服务通信，不直接接 SDK/CAN |
| `runtime.py`, `prompts/system.md` | 多轮调度、图片回传、恢复、预算、模型职责 |
| `__main__.py` | 命令行操作入口 |

任务颜色和槽位由明确指令绑定。VLM 输出的像素框属于选择结果，不是实物颜色检测真值；后续验证服务还需独立检查。只有抓取候选有完整几何、规划通过IK和碰撞检查、持物及放置证据通过，状态才推进。模型说“完成”不会改写任务结果。

## 后续领域服务契约

`services` 示例：`{"grasp":{"url":"http://127.0.0.1:<port>","token_env":"GRASP_SERVICE_TOKEN"}}`。域：`grasp`、`planner`、`robot`、`verification`、`navigation`、`insertion`。具体操作和参数见 `tools.py`，阶段和输出证据见 `mission.py`。

POST `/operations/<operation>`：

```json
{"request_id":"<run-id>:<call-id>","arguments":{"task":{},"target":{},"slot":null,"plan":null}}
```

GET `/executions/<url-encoded-request-id>` 查询同一操作，不重发动作。服务必须按 `request_id` 持久化去重，并在重启后仍可查询；不能以“当前空闲”替代该请求的执行回执。

```json
{"status":"completed","execution_id":"<run-id>:<call-id>","data":{}}
```

状态：`completed` / `accepted` / `running` / `unknown` / `rejected`。物理请求在网络发送前持久化；超时或未知回执阻止后续依赖动作。计算与验证服务本阶段要求同步返回完成结果，尚未提供其异步作业调度。无网络自动重试。真实控制服务应额外执行限位、避碰、控制权、急停、过期计划和标定检查；Agent 校验不能代替这些控制器职责。

下一步依次接入：相机位置确认与同步RGB-D → 内外参/手眼标定 → 电池/槽位目标基准测试 → GraspNet或AnyGrasp服务 → X5规划与受控抓取 → 持物验证 → 导航/对准/插入及稳定验证。每项接入单独测试和实机验收。

## 测试

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s agent/tests -v
```

测试包含严格参数、缺失能力、幂等回执、未知执行恢复、抓取与持物分离、3秒放置验证、HTTP相机与模型流、取消/预算/图片回传。完整比赛链使用明确标注的测试替身；通过不意味着机器人通过实机任务。
