# T2 导航与放置规划交接离线候选

本目录是隔离的协议与规划回放，不是线上部署，不启动真实导航、CAN、机械臂，不注册真实 worker，不发真实 accept 或 complete。

- t2_preview.py：与比赛总控 place 模块相同的 start/inspect/stop/resume 接口；严格隔离比赛 run_id、request_id、父任务 token 与预览子运行 ID；禁止重复请求；取消时等待后台线程退出，不把预览标为已放置。
- recorded_t2_planner.py：使用已保存的 D435 RGB-D 和 FoundationPose 结果，接入已有离线仓口、调平、仓外对齐与浅入规划。D405 输入可选；缺失时在结果中标为 unavailable。
- test_t2_preview.py：身份、取消、重复请求和不可伪造完成测试。
- test_recorded_t2.py：一段历史实拍数据通过 T2 适配器，输出运动学预览但不声称实物完成。
- test_t2_handoff.py：模拟导航协议 point_1 -> point_2，T2 接管后读取预览失败状态；证明不会发出 T2 完成回执。

运行：

```
cd /home/arx/t2-nav-side-20261116
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=$PWD:/home/arx/arx-place-hold-20261116/experiments/placement-path-20261116:/home/arx/arx-place-hold-20261116/experiments/placement-flow-20261116:/home/arx/arx-place-hold-20261116:/home/arx/arx-competition-control:/home/arx/arx-competition-control/web/src:/home/arx/arx-competition-control/web/tests
/home/arx/arx-competition-control/.venv/bin/python -m unittest -v test_t2_preview test_recorded_t2 test_t2_handoff.T2HandoffTest
```

当前预览绝不返回正式放置完成。正式接入还需要实时同帧相机／机器人数据、标定、受控实物执行、松爪与放置验证、失败和恢复语义；只有真实完成后，现有父流程才能完成 T2 visit。不得对历史数据伪造 complete 或把历史放置预览作为可执行指令。
