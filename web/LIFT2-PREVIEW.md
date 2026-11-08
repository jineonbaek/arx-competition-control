# LIFT2S DIY 整机实时预览

9983 首页使用 2026-11-05 交付包的 `lift2_with_sensors.urdf`。27 个 link、26 个 joint 引用 26 个 STL。两个 MID360 共用同一个 STL。头部 D435i 和 Astra Pro 随固定立柱，六个新增部件均保留 URDF 的毫米到米缩放。

## 模型准备

将修正后的交付包 `lift2/` 放在比赛工作区 `references/lift2s-20261105/lift2/`，运行 `python3 scripts/prepare_lift2_preview.py`。也可用 `--package /path/to/lift2` 指定源目录。安装程序核对交付文件的 SHA256，并将网页需要的文件放入 `web/public/models/lift2/diy-20261105/`。CAD 文件由交付包管理，Git 保存入口、代码和校验清单。新的 CAD 修订需相应更新清单。

本机横梁网格 `meshes/body4.STL` 已直接删除原厂网格中实机没有的前支架，包含底脚和顶部安装座。删除的是原版第 [27712, 34130) 个三角面（从零编号），保留的 66836 个面与原件逐字节一致。交付包的 Blender 装配体是交付时的版本，网页读取修正后的 STL。横梁的 visual 与 collision 均引用该 STL。新增传感器及其安装件目前只有 visual，手上 D405 也未建模。

## 反馈和验证

网页从 `/api/state` 获取左右臂六轴和升降反馈。升降显示沿用 41.54 rad/m 名义换算，反馈过期时隐藏相应活动部件。现阶段用于外观和姿态核对，尚未接入碰撞控制。

```bash
node --test web/tests/*.test.mjs
/usr/bin/python3 -m unittest discover -s web/tests -p 'test_*.py'
python web/tests/check_robot_preview_browser.py --base-url http://192.168.58.14:9983
```
