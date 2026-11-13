# 鲲鹏 20 双模式手柄底盘控制与可视化

页面：http://192.168.58.14:9984/ （NUC 本机为 http://127.0.0.1:9984/）。

当前已启用 LT 按住使能的实车控制：右杆前后左右平移，左杆横向自旋。目标满杆 SDK 原值为 2，斜向平移合成幅值不超过 2。先回中并松开 LT，再扣住 LT；XINPUT 的 LT 扳机超过一半启用，松开停车。驱动保留加减速斜坡、反向制动、250ms断流停车和网页停车优先级。浏览器只是显示，关闭浏览器不会代替松开LT。

`server.py` 只读输入，`teleop.py --execute` 通过专用ROS话题控制底盘，默认不带 `--execute` 只预览。不会为手柄再构造第二个CAN SDK实例。

自动识别完整签名后选择 `profiles.py` 中的 NS / XINPUT 配置；未知模式、映射代码变化或多个手柄时报错并禁止输出。模式切换及重连会清除使能，必须重新松LT回中。

- 页面服务： `systemctl --user status arx-gamepad-viewer`
- 控制服务： `systemctl --user status arx-gamepad-teleop`
- 停止手柄控制： `systemctl --user stop arx-gamepad-teleop`
- 启动手柄控制： `systemctl --user start arx-gamepad-teleop`
- 日志： `journalctl --user -u arx-gamepad-teleop`

已集成 NUC 桌面“ARX 一键启动与自检”：双击后依次启动可视化与手柄传输，服务已运行则复用；自检新增两项，故障恢复仅针对异常手柄服务，原硬件恢复确认保留。41项启动集成与原安全测试通过。未设置操作系统登录或开机自动启动。NUC程序目录 `/home/arx/arx-competition-control/gamepad-viewer`；机体驱动仍通过原机体启动流程启动，重启涉及升降归零。

原 NS 程序及实测数据保存在 `legacy/`，XINPUT 本次实测保存在 `xinput_mapping_observed.json` 和两个录制 JSONL。详情见 `TELEOP.md`。

验证： `python3 -m unittest discover -v`（16项），`node --check app.js`。候选驱动3组C++测试通过，真实入口确认过一次全零停车命令。未主动发送非零速度做实车移动测试；方向、制动距离和无线断电的接收器行为仍需现场小幅验证。

手柄两项现作为可选任务异步启动：手柄未接、模式未知、启动或自检失败只显示可选告警，不阻断其他设备的启动与恢复，也不导致整体自检失败。仍保留底盘自身反馈异常时的保护。启动集成测试现为47项；已通过注入手柄启动异常及状态异常验证其他设备正常、硬件进程不变。
