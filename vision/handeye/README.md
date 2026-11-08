# D405 手眼标定准备

当前阶段只下载参考代码、提供 9983 标定入口；没有开始采集、求解或运动。
D405 为眼在手上相机。限位、安装在哪条手臂、标定板类型和尺寸仍待确认。

## 本机参考代码

位置：`/home/arx/arx-competition-control/references/rebot-handeye-52714f61297f`

上游：https://github.com/Seeed-Projects/reBot-DevArm-Grasp
固定提交：`52714f61297f39a7b9f0d40b0a422e38362b800e`

采用稀疏检出，下载 scripts、calibration、drivers、config 和根目录说明；
未下载模型目录，未安装或启动其机器人 SDK。外部代码受根目录 .gitignore 排除。
文件摘要与本机依赖检查见 sources.json。上游代码没有做本地修改。

重点参考：
- scripts/collect_handeye_eih.py：默认自动遍历、等待标记、收集成对位姿；此提交实际列出 25 个姿态，README 的 50 不是当前列表长度。
- calibration/hand_eye.py：OpenCV 刚体手眼求解、结果保存。
- calibration/aruco_pose.py：标记检测及位姿估计。
- drivers/robot/grasp_driver.py：关节反馈经 FK 得到末端位姿。
- drivers/camera/realsense.py：RealSense 采集接口。

## 9983 入口与本轮边界

主页面 D405 相机卡片下方的「D405 手眼标定」打开 /handeye.html。
该页复用现有彩色图像和 GET /api/state；不重新打开相机，不开启深度。
可选择安装侧查看实际关节反馈，默认不猜测安装侧。
显示采集未开始、专用限位/标定板/末端坐标系待确认，自动采集按钮始终禁用。
本轮没有采样或运动后端，修改浏览器的 disabled 属性也不会让该页执行动作。
无需重启 9983 或驱动，刷新页面即可读取静态文件。

## 后续适配时必须确认

- D405 左/右安装侧、相机固定状态及标定板类型/实际尺寸。
- X5 实际关节反馈、FK 模型、关节零点、末端参考坐标系和 TCP 的含义。
- 用户给出的专用限位、可达采样区域、碰撞约束及每次运动后的真实停稳条件。
- 640×480 下正确的相机内参与畸变，图像和实测机械臂位姿的时间对应。
- 采样、求解和独立验证应分别记录，不将求解成功当作实机精度已通过。

不直接使用上游另一台机械臂的 25 个姿态、硬编码时间或 config 中的内参文件。
本机 OpenCV 4.6 有 legacy ArUco 与 calibrateHandEye；上游 ArucoDetector 新接口不可直接照搬。
当前不升级共享 Python/OpenCV，不接管相机或 CAN；之后复用本工作区已有采集和驱动通道。
