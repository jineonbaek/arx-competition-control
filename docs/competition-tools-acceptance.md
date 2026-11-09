# arxgent 比赛工具验收

ROG 验收入口 http://192.168.58.14:9986/ 。使用页面下拉框把示例填入对话框，也可用中文让 Qwen 选择工具。`/tool 名称 JSON` 直接调用相同 ToolRegistry，便于核对参数与结果。OBS 和 JOB 用当前对话返回的编号替换。点击「新对话」会切换记录范围。

本分支使用 main 的 9983 相机采集服务、vision-demo 的视觉模型环境、graspnet-demo 的推理环境。GPU 队列在本验收服务内部串行。图像和作业结果保存在忽略的 agent/runs 内。右腕几何使用 D405、当帧关节和 body4 坐标系。D435 用于 RGB 候选。视觉复核向已配置的 Qwen 服务发送图片。

| 工具 | 手动操作 | 验收证据 |
|---|---|---|
| capture_scene | D435 彩色或 D405 的 rgbd=true | 得到 observation_id。D405 深度需要采集端、右臂和升降反馈有效并停稳，返回采集时间、配对时间差及标定版本。 |
| view_env_state | 空参数，再指定 observation_id | 分别显示最新状态和冻结观测。 |
| locate / segment | 使用上一步 ID 和相机名 | 拿 job_id 查询。核对选框/退回粗定位、复核结论及样本图。在页面末尾输入任务编号查看实际生成的图像。 |
| locate / pixels | 在页面下方输入 OBS、选择 D405，点快照中夹持横梁的实体表面，发送自动填入的 [行,列] | 返回 body4 米坐标。重复查询同一点应一致。无有效深度返回空或明确失败。 |
| locate / triangulate | 在机器人基座静止、目标静止条件下取得不同相机位姿的 D405 快照，标出同一物理点 | 返回三维点和每视角重投影误差；重复快照、过小视差或错误对应应被拒绝。 |
| generate_grasp_candidates | D405 OBS 与同帧复核通过的 segment_job | 得到另一 JOB，查看候选分数、开口、body4 坐标和叠加图。未通过复核或掩膜深度不足时查看失败原因。 |
| get_tool_result | 输入 JOB | queued/running/completed/failed/cancelled/interrupted 及可用结果；分割作业 completed 仍可能需要重新观测。 |
| wait_for_tool | JOB，timeout_s=0 或 5 | 到期返回当前状态，后台作业可继续。 |
| wait_for_tools | 两个 JOB，timeout_s=5 | 使用一个等待预算、按输入顺序返回。 |
| list_tool_jobs / queue_status | 分别调用两个空参数工具 | 返回当前对话的作业及 GPU 资源状态。 |
| cancel_tool | 一个作业正运行，再发起另一个分割，用第二个 JOB 取消 | 排队项变为 cancelled；对运行项返回 running。 |
| flush_queue | 排队多个分割任务后调用 | 清理当前对话的等待项，保留运行项。 |
| move | 手选右臂、给定 body4 米坐标和弧度朝向 | 预览路径的每一点保持同一 Z 和朝向；Z 变化应被拒绝。查看 executed、ik_checked 和 stop_reason。 |
| insert_battery | 选择 D405 RGB-D 观测、仓位与参考系中的水平进仓方向，填入距离 | 查看固定 Z 和朝向的路径、指定行程和未执行状态；仓位轴为输入值，需另行测量。 |
| grasp / release | 分别指定 arm | 返回关闭/打开意图、真实夹爪反馈、未执行原因；实机动作验收需要经验证的运动后端。 |
| hold | 指定 arm、seconds | 在时间窗读取不同时间戳的关节反馈和变化范围。这里检查机械臂稳定性，确认持物还需要目标证据。 |
| finish | 空参数 | 当前任务尚无正确仓位、释放和稳定证据时返回 stage_not_allowed。 |
| list_skills / read_skill | 列出后读取 fixed_z_insertion | 返回版本和经验正文。 |
| publish_insight | 提供 OBS、judgment、reason | 返回同一观测的持久记录编号；无效 OBS 被拒绝。 |

建议第一轮将目标放在 D405 视野且深度有效处，采 RGB-D → segment → 查询分割结果 → pixels → generate_grasp_candidates。把 OBS、两个 JOB、截图和失败原因保留作验收依据。当前前端操作为只读和几何预览。实际抓取需继续验收 X5 夹持参考点、开口、IK 和整臂碰撞。固定 Z 插入还需测量运动中的实际偏差、插入深度与释放后的稳定时间。

手动启动验收服务使用比赛主工作区已有 Python 环境：

```sh
cd /home/arx/arx-competition-tools
/home/arx/arx-competition-control/.venv/bin/python web/src/competition_chat.py --port 9986
```

上游接口参考 Open-RUA/uni-robot-agent 的 1cb4e90a，场景、操作和作业接口均由本仓库实现对接。
