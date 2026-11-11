# 相机、深度与按需视觉识别

三台相机默认只采集彩色画面。9983 的每路深度按钮控制实际深度流，每次启动采集进程都会恢复深度关闭。旧 YOLOE 检测服务、专用依赖和权重已移除。

D435 / D405 的按需识别窗口使用 Grounding DINO、Qwen 和 SAM2；运行入口在 web/src/vision_preview.py，模型环境位于 ROG 的 /home/arx/arx-vision-demo/vision/demo。D405 相机卡片保留提手线段预览，使用 RGB-D、手眼外参和示教偏移生成固定 Y/Z、沿 X 的几何线段，不运行 GraspNet 候选模型。新的比赛实拍参考和尺寸图见 config/field-battery/，共享裁剪参考为 config/battery-reference.png。

## 运行

相机已沿用桌面“一键启动与自检”的相机分支。仅恢复图像可运行：

```bash
python3 scripts/start-console.py --cameras-only
```

`camera_preview.py` 对 D435/D405 使用 `battery_rgbd.py`，Astra Pro 使用 UVC 彩色和 OpenNI 独立深度。三台都导出无损深度 PNG，见 [深度接口、安装与限制](DEPTH.md)。依赖独立安装在 `vision/vendor-python/`，没有修改控制服务的 Python 环境。

重建依赖时使用系统 Python 3.12 对应环境，将 `requirements-rgbd.txt` 安装到该目录（系统已具有 numpy 与 OpenCV）：

```bash
python3 vision/scripts/setup-depth.py
```

本功能不导入 ROS、机器人 SDK 或 CAN，不发送控制请求。机器人和相机的装配 CAD 外参没有被当作实机标定结果使用。绝对精度与模型中心位姿仍需后续测量/配准验证。

参考：
- https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/
- https://dev.realsenseai.com/docs/projection-texture-mapping-and-occlusion-with-intel-realsense-depth-cameras/

## GraspNet 清理（2026-11-11）

已移除候选页面、推理 API、推理脚本和独立 demo 的模型/环境。共享的 web/src/grasp_preview.py 与 web/public/css/grasp.css 继续服务提手预览，名称保留以避免影响已有调用。保留 web/hardware/runtime/grasp-preview 下的冻结 RGB-D、点云及同级 handle-teach-right.json 示教偏移。手眼标定、相机深度按需租约和机器人控制接口不变。

当前抓取方向是视觉定位提手加固定姿态水平前伸。docs/competition-agent-vision-plan.md 中 GraspNet/AnyGrasp 方案属于历史讨论，不再作为当前候选生成入口。FoundationPose 尚需单独验证。
