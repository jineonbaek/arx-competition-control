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

## 本次接入与验证

- 入口：9983 首页最上方；只有旋转/缩放与“重置视角”，不会下发运动指令。
- 独立模块只 GET /api/state，不连接 /events，不引入 TH 的主程序/命令通道，也不读取网页目标值。
- 六轴反馈单位保持 rad，分别映射 left_joint11～16 / right_joint21～26；不使用 CAD 限位裁剪实际姿态。
- 升降映射到 joint4，41.54 rad/m 仅作未标定预览；轮子静态、夹爪 CAD 初始开口。
- 左右臂反馈分别判断新鲜度；升降失效隐藏 body4 及其双臂子树。以 HTTP Date 检查响应时钟，防止整个旧快照回放时仍标为实时。
- 渲染失败只影响预览区；不修改后端、控制区或机器人驱动。
- browser runner 仅测试预览脚本，移除其余控制脚本。资产通过测试代理读取以兼容 Edge 的本地网络权限；保留原始 CSP，未降低真实网页安全策略。

验证命令：
```bash
node --test web/tests/*.test.mjs
/usr/bin/python3 -m unittest discover -s web/tests -p 'test_*.py'
# 开发机安装 Playwright 后：
python web/tests/check_robot_preview_browser.py --base-url http://192.168.58.14:9983
```

2026-11-08：真实浏览器已验证 WebGL + URDF + 21 STL 加载、真实反馈读入、左右臂/升降独立映射、旧快照过期、手机布局、仅 GET 无命令。未执行实机运动核对。
