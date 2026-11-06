# arxgent 网页交互验证

页面：http://192.168.58.14:9983/

已按用户截图将 arxgent 放到右栏顶部，02 / CHASSIS 位于其下方。沿用现有控制台样式；支持发送、连续追问、工具活动、取消、新对话和当前进程内刷新恢复。网页聊天只读。

## Git 与源码

开发工作树：`/home/arx/arx-agent-framework`，分支 `codex/qwen-agent-framework`。

| Commit | 内容 |
| --- | --- |
| b205f33 | 异步聊天API、连续对话和只读权限 |
| ea308ae | arxgent面板、底盘下移、浏览器验证 |
| 6cf3529 | 实时状态、跨轮工具ID、闲置会话回收修复 |

运行源文件冻结于6cf3529。文档记录的后续提交不会改变这些运行源文件。未push或合并。

## 测试证据

- ROG：31/31 Agent测试、89/89网页Python测试、15/15 Node测试通过。
- 开发机Chromium：两轮聊天、刷新恢复、取消、HTML作为文字渲染、390px无横向溢出、聊天输入退出底盘键盘模式通过；无页面异常或运动请求。该项使用离线模型和假控制适配器。
- 真实页面浏览器：输入“先读取机器人实时状态，再查看三个相机，分别描述画面”；实际Qwen3.8-Flash调用get_system_status和observe_scene，返回真实反馈与三路照片描述，单次22.65秒。
- 连续追问名字和此前相机：回答“我叫arxgent。刚才检查的三个相机是d435、d405、astra。”单次10.44秒。
- 刷新后4条消息保留，底盘仍位于聊天面板下面；浏览器0个页面异常、0个控制请求。验证时通过浏览器拦截器禁止/api/command请求实际发出。
- 这些延迟是单次记录，不是性能保证；完整回复采用异步轮询显示，不逐token流式展示。

截图：`C:/Users/jineonbaek/.codex/visualizations/2026/11/06/arxgent-live.png`。截图和原始运行记录未进入Git。

## 运行目录发布边界

当前运行目录：`/home/arx/arx-competition-control`，main仍为e744156。发布前验证已有的server.py、index.html与该Git基线一致。仅以下20个源文件安装为6cf3529内容并逐字节核对：

```text
agent/.gitignore
agent/__init__.py
agent/__main__.py
agent/backends.py
agent/config.py
agent/contracts.py
agent/http.py
agent/mission.py
agent/model.py
agent/observations.py
agent/runtime.py
agent/storage.py
agent/tools.py
agent/prompts/system.md
agent/README.md
web/src/arxgent.py
web/src/server.py
web/public/index.html
web/public/js/arxgent.js
web/public/css/arxgent.css
```

仅重载网页PID3744至277817。相机3643、检测3655、双臂4391/4457、机体4526的进程身份保持；ROS及三组反馈fresh，chassis_active=false。

以下六个既有未提交文件发布前后SHA256一致，没有暂存或修改：

```text
/home/arx/arx-competition-control/README.txt
/home/arx/arx-competition-control/navigation/README.md
/home/arx/arx-competition-control/navigation/scripts/build.sh
/home/arx/arx-competition-control/navigation/scripts/prepare_workspace.py
/home/arx/arx-competition-control/scripts/start-console.py
/home/arx/arx-competition-control/web/tests/test_console_safety.py
```

运行目录的部署覆盖未提交到main：两项已跟踪修改为web/src/server.py、web/public/index.html；新增源码为agent/与web/src/arxgent.py、web/public/js/arxgent.js、web/public/css/arxgent.css。它们的源版本均已在独立开发分支提交。其他既有未跟踪文件保留。

密钥位于运行目录agent/.secrets/qwen.env，权限600、目录700，已被Git忽略；来源开发工作树的私有文件也保持忽略。私有运行文件位于agent/runs/，不由网页静态目录提供。

## 已知边界

聊天不接入机械臂/底盘运动、抓取、导航或插入服务。取消停止后续Agent工作并等待当前云端请求返回，不承诺撤回云端费用。网页进程重启后重新建立聊天会话，已有私有运行证据保留；闲置会话1小时后可回收，每段对话最多20轮。
