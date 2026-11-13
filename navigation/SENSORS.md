# 无车体安装条件下的传感器开发

当前传感器未固定到车体。仅做传感器采集、吞吐、时间戳和离线软件验证，不建立车体TF，不融合两台雷达坐标，不运行机器人运动控制。

## 启动

```bash
cd /home/arx/arx-competition-control/navigation
bash scripts/start_sensors_only.sh profile:=balanced
```

三种档位：

| profile | D435 RGB | D435 深度 | 双MID360S |
|---|---|---|---|
| balanced（默认） | 640×480@30 | 848×480@30 | 每台10Hz、20万点/s、IMU200Hz |
| high_speed | 640×480@60 | 848×480@90 | 同上 |
| max_resolution | 1920×1080@30 | 1280×720@30 | 同上 |

另开终端计数：

```bash
cd /home/arx/arx-competition-control/navigation
bash scripts/check_sensor_rates.sh 30.0
```

默认使用域192、LOCALHOST和专用Fast DDS传输配置。算法订阅者应先source sensor_env.sh，使用相同域及传输配置；不把这些设置应用到现有机体进程。当前XML使用128MiB共享内存段、本机UDP和4MiB socket缓冲上限，无全局sysctl或DDS配置变更。若设备或IP变化，需更新配置；有线网卡的192.168.1.50/24是临时接收地址，重启后可能不存在，脚本会检查。

## 官方参数

MID360S典型10Hz、首回波200000点/s；水平360°，垂直约-7°到52°；100BASE-TX；内置ICM40609 IMU。官方系列协议定义IMU输出率200/500/100/50Hz，当前保持200Hz。

D435深度最高1280×720，标准宽视场90fps档为848×480；RGB官方1920×1080@30fps。本机SDK枚举另外支持640×480 RGB@60fps。不同分辨率的帧率上限不能混用。90fps不适用于1280×720深度或1920×1080 RGB。

官方来源：
- https://www.livoxtech.com/mid-360s/specs
- https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html
- https://www.realsenseai.com/products/stereo-depth-camera-d435/
- https://dev.realsenseai.com/docs/high-speed-capture-mode-of-intel-realsense-depth-camera-d435/
- https://github.com/realsenseai/realsense-ros

## 本次定位与改进

默认Best Effort订阅在15fps配置下仅收到约6Hz图像；Reliable订阅恢复约15Hz，30fps档C++和Python均约30Hz。两台IMU的轻量C++接收率均约200Hz，旧Python测试169/171Hz不是硬件上限。

仅设置FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA未解决本机Best Effort图像丢帧。改为显式配置足够大的共享内存段和本机UDP后，Best Effort也接近所有测试档位的标称。所有吞吐测试期间两台MID360S同时采集，点云和IMU保持目标速率。数据内容未导出；计数器只保存频率/时间统计。

短时高帧率结果：RGB640×480约59.53Hz、深度848×480约89.87Hz。最高分辨率RGB1920×1080与深度1280×720均约29.98Hz。完整CSV、启动日志与版本在reports/latest-throughput-path.txt指向的目录。

最高分辨率测试启动阶段有一次global timestamp polling资源忙日志；采集帧率达标，不能据此认定硬件同步已验收。CSV的header_age包含相机采集时序、雷达积帧和时间基准因素，不等于纯网络延迟。点云点频统计包括原始数据点，不代表每个点均为有效回波。

## 安装暂缓期间的流程

1. 优先稳定balanced档并进行更长时段、插拔恢复和处理节点并发测试。
2. 在NUC本地进行有界录包与回放，比较录包前后接收率和时戳间隔。图像/点云导出另需用户明确授权。
3. 检查雷达自身IMU单位、点云线号/逐点时间、各传感器独立身份；可以在传感器原生坐标查看数据，但不假设它们共址。
4. 相机可先验证深度有效率、相机内部RGB/深度对齐和静态目标检测。启用对齐、滤波或点云后重新测量帧率。
5. 底盘网关使用假适配器离线开发和测试；真实驱动、车体外参、移动SLAM与实车导航留待条件具备后实施。

启动只包含RealSense和Livox驱动。Ctrl+C结束。控制/机械臂进程不由这些脚本启停。

## SDK退出兼容修正

持续测试时发现官方SDK2内置的旧spdlog模板符号与ROS系统spdlog发生动态符号冲突，表现为采集正常而退出时SIGSEGV。gdb栈显示SDK logger析构进入了系统libspdlog的实现。

已在本导航目录的SDK构建中增加链接导出表：保留官方头文件的63个C API，隐藏私有C++/spdlog符号。`scripts/generate_sdk_exports.py`在构建时生成config/livox_sdk_exports.map，scripts/build.sh应用该链接选项；不修改官方源码检出或系统日志库。修复后gdb启停验证正常退出，整套采集入口又完成20秒复测，两台驱动均正常退出，图像约29.98Hz、双雷达点云约10Hz、IMU约200Hz。
