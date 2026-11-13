# 臂上MID360S单雷达建图测试

2026-11-04。当前在线雷达192.168.1.3，序列号ARMCPA80036301。用户确认它已安装在最终固定位置，即连接机械臂的可升降平台；顶部另一台雷达网线已拔掉。本轮仅调用该雷达和FAST-LIO，不启动Nav2、底盘/机械臂驱动或发送任何机器人运动/停车指令。

## 当前测试

原生点云与本机内置IMU → FAST-LIO → 雷达/IMU自身里程计与三维点云地图。

坐标链：lidar_odom → lio_imu → livox_frame。没有创建base_link变换，没有使用尚未测量的车体外参；地图原点是本次初始化的IMU参考，不是车辆全局定位原点。

内部LiDAR–IMU标称外参已按官方Mid360S手册第22页确认：IMU在LiDAR坐标为[0.011,0.02329,-0.04412]m、轴向相同；FAST-LIO使用反向平移[-0.011,-0.02329,0.04412]m及单位旋转。驱动保持额外点云变换为零。参考：https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/UM/20260601/Livox_Mid-360s_User_Manual_en.pdf

初次初始化应保持雷达平稳；第一轮尽量保持升降高度不变，以便区分底盘运动与升降运动。后续若平台升降，本原生LIO估计的是传感器运动，不能直接解释成底盘运动。FAST-LIO本身无全局闭环，本次不声称完成导航定位或移动精度验证。

## 运行与数据

- 启动脚本：scripts/start_arm_lidar_mapping.sh。运行时文件锁避免重复开启；每次启动生成新的报告目录和地图文件。
- 当前报告目录：reports/latest-arm-mapping-path.txt。
- 实时状态：当前报告目录/live-status.json，每5秒更新。
- 原始点云：/mapping_test/lidar；IMU：/mapping_test/imu。
- 配准点云：/mapping_test/cloud_registered；地图：/mapping_test/map。
- 位姿：/mapping_test/odometry；轨迹：/mapping_test/path。
- 地图别名：maps/arm_lidar_latest.pcd，指向本轮报告目录/map.pcd，每60秒原子覆盖保存。
- 手动保存：source sensor_env.sh 后，执行 ros2 service call /mapping_test/map_save std_srvs/srv/Trigger {}。它仅保存地图。

## NUC上查看

```bash
cd /home/arx/arx-competition-control/navigation
source sensor_env.sh
rviz2 -d config/mapping_test/arm_mapping.rviz
```

固定坐标系为lidar_odom。此配置未自动打开GUI，也未将环境点云或图像导出本机。

## 仅结束本轮建图

```bash
cd /home/arx/arx-competition-control/navigation
source sensor_env.sh
python3 scripts/stop_arm_lidar_mapping.py
```

停止脚本先核对PID命令行，只请求保存地图并结束本轮建图launch，不发底盘停车或其他运动指令，不启停现有机体驱动。

## 源码适配与检查

FAST-LIO增加默认关闭的publish.sensor_only_tf选项。该测试开启时发布原生IMU TF并正确标记body cloud，跳过原来的base_link查找；标准车体路径默认不变。地图按当前有效KD-tree点发布，避免无限重复拼接；PCD写入先写临时文件再原子替换，并检查空地图/写入失败。

已完成编译、真实点云/IMU初始化、地图流、PCD读回与有限坐标检查，以及实际ROS发布话题审计。各测试节点没有机器人运动指令发布者。尚未完成动态扫描精度、回环、底盘定位、升降外参或Nav2验收。


## 雷达断电恢复（2026-11-04实测）

雷达断电后不能让旧FAST-LIO直接接续使用。实测网络恢复但旧滤波器位置严重发散。应先停止本导航建图launch，保留断电前地图，再由操作员确认底盘和升降静止后开启新建图会话。当前尚无自动断流保护，不支持无缝续图。

断电前有效地图备份：maps/arm_lidar_before_power_cycle.pcd；旧195747会话已用INVALID_AFTER_POWER_CYCLE.json标记断电后的地图无效。202545为重新初始化的新会话，与旧图坐标不连续。
