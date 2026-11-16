# Automatic Placement Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 将既有抓取机制扩展为单红圈左侧放置，先完成隔离软件验证。
**Architecture:** 纯RGB-D几何模块、固定状态机、复用BatteryTask/Controller的执行适配、同风格页面；均无导入副作用。
**Tech Stack:** Python3.12, NumPy, OpenCV, SciPy, existing ROS controller, HTML/JS.
**Spec:** docs/superpowers/specs/2026-11-14-automatic-placement-design.md

## Global Constraints
- 不启动设备/服务，不推送，不修改原工作树。
- 左侧base_link +Y；最大XY30mm/朝向8°，Z3mm；边界不足自动收紧或拒绝。
- 同一相机冻结帧及实测深度；不以红色像素或圆心直接冒充3D落点。
- 失败/取消不松爪、不重放动作；只有新观测的稳定电池证据可完成任务。

## Review Focus
- 红色实心物体、多圈、缺失/零/旧深度不能产生执行落点。
- 斜面或左侧已占位不能放置。
- 支撑高度与XY容差分离，不能宽容差放空。
- 取消/重启/回执未知不能重放或松爪。
- 网页只读打开，动态文本不作为HTML。

### Task 1: 红圈深度与落点
Files: agent/placement_geometry.py, agent/tests/test_placement_geometry.py
Interface: analyze(frame, policy, now) -> dict; fit support plane, metric circle, left whole-footprint target.
- [x] 写合成RGB-D测试并验证RED。
- [x] 实现几何、数据绑定与边界检查，运行测试GREEN。

### Task 2: 固定流程与执行适配
Files: agent/placement_flow.py, agent/placement_backend.py, agent/tests/test_placement_flow.py
Interface: PlacementFlow(store, backend).run(); backend observe/prepare/move/release/verify/stop.
- [x] 先测试阶段、取消、错误、不得提前松爪及重启不重放。
- [x] 实现固定流程，复用RunStore日志、轨迹、反馈和BatteryTask控制权。
- [x] 新配置默认禁用运动；真实持物证据、CAD和策略缺失时明确拒绝。

### Task 3: 页面接入与验证
Files: web/src/placement_preview.py, web/src/server.py, web/public/placement.html, web/public/js/placement.js
- [x] 接入显式可选后端、只读GET、经会话/Origin保护的POST。
- [x] 页面显示两相机、红圈、圈内左目标、深度/容差、阶段/日志/取消。
- [x] 运行聚焦测试、已有抓取回归、完整软件套件；记录未通过项目与实机未验证范围。
- [x] 独立检查差异，不合并、不推送、不部署。
