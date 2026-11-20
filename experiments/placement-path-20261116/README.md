# 电池调平与浅入离线规划

该目录为只读离线回放，不连接 ROS、CAN 或机器人驱动，输出一律标记不可执行。

流程：读取 D435 保存的 RGB-D 帧和同帧电池姿态报告，识别仓口；计算电池调平建议；以观察到的持物变换计算有效 TCP；生成左侧入口几何目标和 5 mm 固定 Z 浅入目标；生成仓外对齐和浅入的关节轨迹预览。D405 如有与 D435、手臂和机体时间配对的 RGB-D 帧，优先用于估算上下位置偏差，重新计算名义仓口余量；D405 的上下偏差不是电池倾角，电池倾角仍使用 D435 观测。

重放示例：

    cd /home/arx/arx-place-hold-20261116/experiments/placement-path-20261116
    export PYTHONPATH=$PWD:/home/arx/arx-place-hold-20261116/experiments/placement-flow-20261116:/home/arx/arx-place-hold-20261116:/home/arx/arx-competition-control/web/src
    p=/home/arx/arx-place-hold-20261116/reports/d435-visual-level-active/estimate-1794782014072959960
    /home/arx/arx-competition-control/.venv/bin/python run_replay.py --d435 "$p/frame.npz" --pose "$p/pose.json" > replay.json

可选参数：--d405 配对深度帧文件，--mode demo 或 competition，--right-offset 0 到 8 毫米。需要提供机器人的已记录关节、升降、相机变换、模型配置和 D435 姿态结果；离线回放不证明标定精度或当前位置。

historical-replay.json 为过去的 D435 录像结果：电池倾角约 0.552 度，预览目标浅入约 5 mm，Z 范围为 0；没有找到可用的同时间 D405 深度帧，不能据此验证双相机。该帧的名义模型净空约 1.22 mm，仅为模型计算，不代表实物余量；关节轨迹仅检查现有模型与立柱，未完成双臂、夹爪、负载和环境的完整碰撞验证。无自动执行入口。
