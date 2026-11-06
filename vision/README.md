# 左侧电池把手只读识别

此版本针对提供的白色打印电池、暗色桌面和当前可见把手，使用多亮度阈值、孔形几何与模型尺寸约束自动选择图像最左侧的候选。它不是训练完成的通用电池分类器，也不执行完整 CAD 六维位姿配准；换颜色、强遮挡或其他带孔白色物体可能导致漏检或误检。

- 蓝色 H：识别到的把手孔中心，孔内可能看见背景，禁止直接将其深度作为把手深度。
- 绿色 G：由孔几何确定的横梁实体表面采样区域，使用有效深度的稳健统计值定位。
- 黄色 G：只有二维候选，三维定位无效。
- XYZ：D435 彩色光学坐标系，X 向图像右，Y 向图像下，Z 向前，网页显示毫米。
- 这是表面位置，不是 CAD 中厚度中心，也不是机器人基座坐标或经过碰撞验证的抓取目标。

D435 同时采集 640×480、15 FPS 的彩色与深度，通过 RealSense SDK 将深度对齐到彩色，使用相机出厂内参和深度单位反投影。帧间时间差大于 40 ms、处理/帧时效超限、有效深度不足或孔宽尺度明显不符时不输出三维点。未启用填洞来伪造缺失深度。

标注直接烘焙到对应帧。JSON 指向唯一命名的 JPEG，避免图像和坐标错帧；仅保留最近 35 张带标注图像，不记录视频。

## 运行

相机已沿用桌面“一键启动与自检”的相机分支。仅恢复图像可运行：

```bash
python3 scripts/start-console.py --cameras-only
```

`camera_preview.py` 对 D435 使用 `battery_rgbd.py`，其他两台继续彩色预览。依赖独立安装在 `vision/vendor-python/`，没有修改控制服务的 Python 环境。

重建依赖时使用系统 Python 3.12 对应环境，将 `requirements-rgbd.txt` 安装到该目录（系统已具有 numpy 与 OpenCV）：

```bash
python3 -m pip install --no-deps --target vision/vendor-python -r vision/requirements-rgbd.txt
```

本功能不导入 ROS、机器人 SDK 或 CAN，不发送控制请求。机器人和相机的装配 CAD 外参没有被当作实机标定结果使用。绝对精度与模型中心位姿仍需后续测量/配准验证。

参考：
- https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/
- https://dev.realsenseai.com/docs/projection-texture-mapping-and-occlusion-with-intel-realsense-depth-cameras/
