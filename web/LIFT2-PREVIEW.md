# LIFT2 只读整机预览

官方模型：ARXroboticsX/ARX_Model 的 LIFT2/lift2.7z；厂家现场沟通截图确认选 LIFT2。
模型以原始 lift2.urdf + 21 个 STL 加载。显示方式沿用 TH：Three.js、URDFLoader、STLLoader、OrbitControls；不导入 TH 控制界面、SDK、WebSocket 或命令逻辑。

来源固定在 models-lift2.json。下载资产忽略于 Git；没有提交二进制模型、SDK 或运行日志。
首次准备（Linux 需已有 7z/7zz/bsdtar；Windows 可用系统 tar）：
```bash
python3 scripts/prepare_lift2_preview.py
python3 -m unittest discover -s web/tests -p test_lift2_assets.py -v
```
不需要重启 9983 或机器人驱动。全部浏览器依赖本地提供，不依赖 CDN/importmap/内联脚本。

右臂/左臂各六轴使用 /api/state 的实际 rad 反馈；不跟随滑块或目标角度。
CAD 中夹爪是米制直线关节，本机反馈是 SDK 原值：尚无可靠对应关系，本阶段固定 CAD 初始开口并明确标注。
升降模型 joint4 的行程是 0～0.46 m。电机角反馈暂用 41.54 rad/m 的现有名义比例做视觉换算；这不是实测高度或控制标定。
不据模型 URDF 的关节范围修改实机软件限位，不钳制实际关节反馈，不接碰撞控制。

## CAD measurements

Measured directly from the official binary STL coordinates (meters, consistent with URDF):

| Mesh | Local bounding-box size |
|---|---|
| base_link.STL (includes fixed column) | 571.0 x 657.2 x 932.2 mm |
| body4.STL | 433.2 x 600.0 x 417.7 mm |
| wheel1.STL | 64.0 mm axial width, approximately 150.0 mm diameter |

The joint4 URDF limit is 0.46 m. These are CAD measurements, not physical measurements or a motor-to-linear transmission calibration.
