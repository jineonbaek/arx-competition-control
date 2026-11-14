# 整场比赛框架（第一版）

入口：9983 `/map.html`。目标固定为无人机指定左上角电池，放入充电柜红框 A 口，无口令解析。

顺序：`navigate_pick → pick → carry → navigate_place → place`。`place` 的完成结果应覆盖插入、释放、撤指、稳定至少 3 秒。导航任务的完成结果必须代表到工作区且已停稳。

当前只注册真实 `pick` 适配，直接复用现有抓取工作流、固定点位设置、动作和容差。它使用整场控制权，结果仍在原抓取详情页展示。其余任务均未接入；开始比赛会明确停在首个未接入任务，不能借网页标记完成来启动机械臂。模拟任务仅存在于测试，生产网页没有模拟完成入口。

## 模块接口

在服务端构建时调用 `competition_task.register(stage, module)`。这是 Python 内部注册，不预定雷达同事的 ROS/HTTP 消息格式，也不修改导航代码。

模块实现以下四个方法：

- `start(context)`：同一 `request_id` 只能启动一次。先检查 `context['cancelled']`；发动作前再次检查。返回 `status` 为 `accepted`、`running`、`completed`、`failed`、`stopped` 或 `unknown` 的字典。
- `inspect(context)`：读取同一任务的真实状态和回执，不发送运动。不能将旧回执误作当前动作的完成。继续前应检查当前位置、持物、工作区等本任务所需条件；信息不足返回 `unknown`。
- `stop(context)`：幂等停止本任务，保持夹持，不回零、不下电；完成清理后返回 `{'stopped': True}`。不得在返回后又发送运动。停不下来返回未确认，控制权不虚报已释放，网页允许重试停止。
- `resume(context)`：仅在 `inspect` 返回 `{'status': 'stopped', 'can_resume': True}` 时调用。沿原目标完成剩余部分，不能重跑已完成动作；没有恢复能力不能声明 `can_resume`。

`context` 包含 `run_id`、`stage`、稳定的 `request_id`、进程内 `token`、`threading.Event` 类型的 `cancelled`、现有 `controller` 和 `environment_clear`。token 不通过网页或日志序列化；整场完成前模块不得调用 `release_motion`。导航适配需要使用总控授权的调用入口，不能以另一网页会话抢占。

雷达同事负责：去指定工作区、到位停稳通知、停止导航。未来适配把这些转换成上述任务状态即可；同一任务的通知必须和 `request_id`/工作区匹配，旧通知不能推进新任务。

## 异常与继续

正常顺序不调用 Qwen。模块内部已有的识别重试、视觉纠偏、导航重规划保持原逻辑。

异常时，总控向 Qwen 提供当前阶段、原任务状态、完成记录和机器人反馈。Qwen 通过已有 `QwenClient` 只选择已注册恢复任务；判断不明确、没有能力、模型调用失败均停止推进并清理控制权。模型调用本身不执行动作；停止后迟到的模型答复被丢弃。

具体恢复动作先不实现。模块可以通过 `recoveries = {'名称': recovery_module}` 注册恢复任务，接口同上。恢复完成后必须再次读取原任务状态，不能单凭模型声称成功推进流程。

停止后保留完成记录。“检查并继续”重新获取控制权、读取当前反馈并核对中断任务：已完成则进入下一任务；仍在执行只查询原回执；明确可恢复才调用 `resume`；否则进入异常分析。完成动作不重新 `start`。抓取中断阶段的具体恢复尚未接入，不伪造继续能力。

整场运行及异常恢复期间总控保有控制权；整场完成、主动停止或无法恢复失败后交还 9983。实际停止未确认时显示“控制权待清理”，不能假称手动已经可用。

任务日志位于 `agent/competition-runs/`，不提交运行记录。第一版不做跨服务重载/重启恢复，不自动恢复运动；重载后是新的待机总控。

HTTP：`GET /api/competition-task` 查询；`POST /api/competition-task` 的 `action` 为 `start`、`stop`、`continue`，均带 `request_id`。开始/继续还携带当前 `expected_epoch` 和沿用已有的 `environment_clear=true`。身份和 Origin 检查复用现有控制台；重复请求不重复启动。

当前仅完成框架和模拟验证，不代表载物过障、运输姿态或插仓已经实现或实机验收。
