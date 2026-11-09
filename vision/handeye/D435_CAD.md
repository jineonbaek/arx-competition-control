# 头部 D435 的 CAD 外参初值

目标设备为 D435（349622074226）。交付包的 D435i 外壳通过后方 45 mm 孔距固定到头部。交付文件 docs/LIFT2_双相机装配参数.json 给出 STL 光轴在 base_link 中朝向 (0.674131, 0, −0.738612)，与 URDF 一致。官方 D435i xacro 复用 D435 宏并增加 IMU。此结果用于只读位置预览，尚未用已知机器人坐标的实物点验收。

以 STL 局部坐标为准：+Z 朝镜头前方，+X 朝车左，+Y 朝机壳顶部。底部三脚架孔口的 24 个网格顶点组成半径约 3.675 mm 的圆，中心约 (44.9054, 0.6534, 10.1523) mm。前面板最大 Z = 25.0523 mm，与孔口 Z 相距 14.9000 mm。官方 D435 xacro 给出孔口到深度光心沿左、上、前分别为 17.5、12.5、10.6 mm（前向 14.9 − 0.1 − 4.2）。从光学系到 STL 的旋转是 diag(−1, −1, 1)。彩色到深度变换使用该序列号 SDK get_extrinsics_to 的列主序旋转及米制平移。完整矩阵见 results/d435-cad-349622074226.json。

相机固定在 base_link 一侧；换到随升降移动的 body4 时，按 joint4 的 URDF 原点及现有名义升降比例换算。SDK 零位和升降比例的不确定性只影响 body4 的换算。识别单个 55 mm AprilTag 中心无需假设整板的码间距和编号布局。单帧重投影误差度量图像拟合，不能检验整车外参的绝对准确度。

[官方 D435 模型](https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_description/urdf/_d435.urdf.xacro) · [官方 D435i 模型](https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_description/urdf/_d435i.urdf.xacro)
