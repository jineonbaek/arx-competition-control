# 导航与同机任务通信 v1

本次只修改 navigation。`agent/battery_*`、web、9983服务、硬件驱动均不改。
导航只传 `point_1` 或 `point_2`，不决定抓取、放置或其他操作。
**现有未接入协议的 agent 不会自动接到地点通知。** 未登记任务调用者时，含任务点路线禁止开始，普通停车点和途经点不受此限制。

## 路线点

- 途经点 `via`：按顺序必须经过，默认车心容差0.10m，可用JSON设置0.02～0.5m。连续规划至下一任务/停车点，中途不主动停车。最后一个via仍需作为终点停车。
- 普通停车点 `stop`：沿用当前原生MPPI到达、位置0.045m/航向0.09rad及约1秒实测停稳检查。
- 任务点 `task`：到达并停稳后通知地点1/2；等待任务完成且返还控制权，再从新的实测位置继续。

RViz多点面板：选类型→添加并拖箭头→核对列表和标记→开始路线。可上移、下移、删除、保存路线、载入为草稿。运行中不允许编辑；载入不自动执行。地图身份不符时需现场核对坐标。

## 端口与控制权

- 通信API只监听 `http://127.0.0.1:9985`，在导航唯一执行进程内运行。不发布SDK/CAN运动接口。
- 原 `http://127.0.0.1:9983` 控制台保持原样。
- 任务仍通过它自己的现有Controller流程获取/归还底盘控制权；导航无法为未修改的agent创建或验证其进程内 `_MotionToken`。
- 交接期间导航不发速度，也不反复制动或纠正旧停车位。新箭头会被拒绝。
- 完成后导航独立确认实际控制台 `chassis_active=false`、`motion_reserved=false`、反馈新鲜及新一段停稳证明；两张图和当前全车轮廓也须安全。
- 这是合作协议：任务维护方必须遵守自身运动互斥、停止和归还规则。本改动不宣称能阻止绕过9983直接打开CAN的其他进程。

## 1. 注册和保活

```http
POST /v1/worker
Content-Type: application/json

{"worker_id":"battery","instance_id":"调用进程的UUID","point_ids":["point_1","point_2"]}
```

响应 `{"token":"...","lease_seconds":2.0}`。保存该调用进程的instance_id；每0.2秒建议调用一次：

```http
POST /v1/heartbeat
Authorization: Bearer <token>
Content-Type: application/json

{}
```

连接租约2秒。心跳只证明调用者活着，不证明动作完成。整个任务默认600秒期限。断线/过期/任务失败使导航暂停，不跳点或自动重跑。

同一进程重连可使用原instance_id续期；已进入needs_attention的任务不能靠重连自动恢复，只能确认停止并取消。导航进程重启后的未确认交接保持needs_attention，不自动重放。

## 2. 获取到点事件

`GET /v1/task`，带Bearer token。无事件返回 `{"task":null}`。有事件：

```json
{"task":{
  "mission_id":"本轮路线UUID",
  "visit_id":"本次到访UUID",
  "point_id":"point_1",
  "route_point_id":"该路线点ID",
  "map_id":"地图身份",
  "goal_pose":[1.0,0.0,0.0],
  "arrival_pose":[0.99,0.01,0.01],
  "pose_stamp_ns":123456789,
  "status":"notified",
  "revision":1,
  "navigation_ready":false,
  "control_released":false,
  "error":null
}}
```

位置为map坐标系米，角度为弧度，+X前/+Y左/+Z上。`point_id`只用于告诉任务地点，不能拿路线序号代替；重复访问同一地点会有不同visit_id。

## 3. 接管、完成、失败

```http
POST /v1/task
Authorization: Bearer <token>
Content-Type: application/json

{"request_id":"每次事件的UUID","mission_id":"...","visit_id":"...","point_id":"point_1","event":"accept"}
```

任务程序确认自己的控制权后，使用现有方式处理该地点（可移动底盘）。处理过程中继续保活。

成功且已停车、归还控制权、恢复可导航状态后：

```json
{"request_id":"另一个UUID","mission_id":"...","visit_id":"...","point_id":"point_1",
 "event":"complete","control_released":true,"navigation_ready":true}
```

这两项true必须来自任务实际结果，不能在finally里无条件填写。导航收到后进入 `resume_pending`，实际停稳/控制权检查通过后才记录 `acknowledged`、推进路线。仅线程退出或HTTP200不是完成。

失败：`event:"fail", error:"原因"`。取消后的停止回执：`event:"cancelled", control_released:true`。

- 相同request_id、相同完整内容重发返回同一回执；相同ID改内容409。
- 旧mission/visit、错地点、取消后迟到complete的新请求均拒绝，不能推进新路线。完全相同的历史request_id可返回历史回执，但不会改变当前visit。
- POST超时使用原request_id和原内容重试或查询，不创建新ID重复作业。
- `GET /v1/state`可不带token检查协议版本、可用地点和当前任务状态，不包含token。
- 只允许一名合作任务调用者。另一个instance不能接管未确认的交接。

## 取消和重启

RViz“取消并清空”取消精确的本次visit，并通过现有底盘停止入口请求一次停车。控制权未清理、停稳未核验前保持阻止导航。之后只有新的人工箭头/路线开始才可能运动；取消不等于任务成功。

调用者发现status=cancelling时停止自己的动作、清理任务控制权，再发送cancelled回执。若调用者故障，操作员显式取消后，导航仍需实际控制台空闲及实测停稳才关闭交接记录；不会报告任务完成。

## 调用端接入

导航目录提供 `task_client.py`，没有任何CAN或机械臂操作。可从任务调用入口导入：

```python
from task_client import TaskClient
client = TaskClient()
client.register(["point_1", "point_2"])
# 独立保活循环每0.2秒 client.heartbeat()
context = client.task()
# 对notified：原有任务获取Controller控制权，然后发送accept。
# 处理 context["point_id"]；具体动作由任务自行决定。
# 真正结束并归还控制权后，使用原身份发送complete或fail。
```

`examples/task_caller.py`给出依赖注入模板，必须显式提供任务处理函数；默认不登记、不抓取、不伪造完成。真实agent接入由它的维护方在自己的调用入口使用本协议；本次不改它的文件。

## 软件验收范围

已实现内容应以本次报告为准：纯状态/HTTP、ROS隔离任务交接、真实Smac/MPPI虚拟底盘及RViz插件构建。未接真实worker、未实车执行抓取或导航时，不得把隔离结果写成实车验收。


## 9985交接监控与人工联调页面（2026-11-14）

导航执行节点启动后，打开 `http://127.0.0.1:9985/`。在Mac通过SSH转发同一地址访问：

```bash
ssh -N -L 127.0.0.1:9985:127.0.0.1:9985 arx@192.168.58.14
```

先尝试局域网；连接不上再将SSH目标换为 `arx@100.102.103.88`。不改变服务的127.0.0.1监听，不开放公网。9985随导航执行进程运行；只有建图、未启动执行端时可能不在线。

默认只读，显示到点通知生成、通知接口被读取、确认接管、收到完成信号、导航确认，以及等待控制权/新鲜数据/停稳/轮廓检查的原因。`notified`仅表示通知已生成，`accept`才表示调用方确认接管；完成HTTP请求返回不等同于已恢复导航。

人工联调时点击“启用本页人工联调”，登记独立调用身份并按原协议保活。真实总控在线时本页只读，不能代发其回执。到点后确认接管；作业真实完成后，明确勾选完成、停止运动并归还控制权、可导航状态，才可发送完成信号。随后仍由原导航线程验收并推进剩余路线。页面不执行抓取、不生成虚假到点，也不提供强制放行。

页面支持报告失败和在导航已经请求取消后确认停止/归还。超时或5xx保留原UUID及完整内容重试；明确的协议拒绝单独显示，不让该回执永久阻塞取消或下一次到访。只读进度记录最多128条，保存在原导航交接journal中，不包含worker token或instance_id。

本标签页sessionStorage保存自身联调身份和待重试请求；刷新不会自动登记或发送完成。浏览器复制标签页可能共享同一人工会话；不将它用于多人并发控制。后台冻结/断网可能导致2秒租约过期，按原规则暂停，不能保证浏览器后台持续保活。

## 9983 总控接入及任务点预检（2026-11-15）

上文“本次不改web/agent”的表述记录最初导航协议交付范围。当前9983总控已经复用本协议，现场比赛统一从9983/map.html开始，勿同时从RViz再次开始或在9985页面登记第二个worker。导航仍只提供地点身份，不调用或改写抓取/运输/放置模块。

- T1必须为`type:"task", point_id:"point_1"`，T2必须为`type:"task", point_id:"point_2"`。名称、行号、via/special不能代替业务身份。比赛要求任务身份序列恰为T1→T2各一次；通用导航仍允许重复访问任务地点。
- RViz“添加当前车位与朝向”或拖箭头均可记录这两类点。坐标为map下底盘车心的米/弧度位姿，必须现场确定；当前没有确定实际工作区，本轮不填入测试坐标。
- 保存路线定义保留map_id、route_id和各route_point_id；载入始终为草稿，不自动开始。完成或取消后重测须明确重新载入，不能用start伪装继续。
- 9983整场占用`competition_reserved`与作业占用`motion_reserved`不同。导航仍检查作业占用/底盘活动、执行接口、反馈、数据、全车轮廓、停稳与执行门；不会把整场占用当作绝对禁行，也不把它当作绕过保护的许可。
- 末次控制台查询之后再次核对几何/停稳使用的同一位姿源仍有效且未改变。执行门关闭时保持resume_pending，不提前acknowledged、消耗任务点或丢失下一段。

现有`GET /v1/state`新增只读`route`对象，包含当前route_id/map_id、state/running/index/point_count、task_points（原列表零基index、point_id、route_point_id、pose）、competition_config_valid、competition_startable_draft、issues。初始化尚未发布路线时为null。route只用于诊断，不写journal、不修改路线、不登记worker。

`competition_startable_draft=true` **仅表示任务身份正确且路线为草稿**，不是允许运动的判断。仍须总控按原流程登记worker并由导航检查实时状态；worker_ready=false在比赛尚未开始时是正常待机状态。

只读检查工具，不会注册、启动或发送任何回执：

```bash
python3 polarbear_adapt/check_task_route.py --file /绝对路径/route.json
python3 polarbear_adapt/check_task_route.py
```

默认只GET本机127.0.0.1:9985；也可通过`--url http://127.0.0.1:转发端口`检查SSH私网转发。返回码0表示路线配置可作为比赛草稿；2表示配置/状态不满足或连接失败。工具不确定真实工作区是否可达，不验证抓取成功，不触发place。真实place未接入时由总控明确停止，导航不伪造成功。
