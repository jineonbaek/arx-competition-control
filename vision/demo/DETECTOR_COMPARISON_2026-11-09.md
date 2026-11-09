# YOLOE 与 DINO 离线对比，2026-11-09

## 数据与条件

从之前 DINO 验证和实时抽样保存的原图中，排除完全重复和相邻几乎相同的帧，选 6 张 640×480 图片。人工标注两张正样本中的 3 块电池（两块白色打印件、一块深色实物）；另 4 张为地面、夹爪近景、空画面等无可辨认目标场景。远处物体难以确认的室内场景未计分。每张只评估可见部分的外框，IoU≥0.5 算命中，一对一匹配。

两条路线都使用灰度场景。DINO 使用当前部署的形状提示词；YOLOE 使用新实拍裁剪构成的三视图参考、或分别使用三个相机配置里的单张实拍参考。所有参考均来自新实拍，旧白色打印件参考未用于初始化。指标不包含参考图自匹配。相同的 0.20 置信阈值仅用于并列报告，两种模型的分数不视作可互换概率。

DINO 使用现有 CUDA 环境。原始 YOLOE 环境是 PyTorch 2.15.0+cpu / Ultralytics 8.4.202，使用 4 个 CPU 线程。每图预热后连续测 3 次取中位数，再报告六图中位数；不计加载、Qwen、SAM2、相机和 USB 时间。两者设备不同，耗时不能用于推断同硬件模型速度。

## 检测结果

| 配置 | 命中 / 3 | 漏检 | 未匹配候选框 | 六图推理中位数 |
|---|---:|---:|---:|---:|
| DINO tiny，GPU，阈值 0.20 | 3 | 0 | 32 | 216.3 ms |
| YOLOE-11s，三视图参考，CPU，阈值 0.20 | 0 | 3 | 0 | 112.0 ms |
| YOLOE-11s，drone 单参考，CPU，阈值 0.20 | 0 | 3 | 0 | 106.2 ms |
| YOLOE-11s，rack 单参考，CPU，阈值 0.20 | 1 | 2 | 0 | 113.0 ms |
| YOLOE-11s，basket 单参考，CPU，阈值 0.20 | 0 | 3 | 0 | 98.3 ms |

YOLOE 三视图阈值降低到 0.10 后，命中 1/3，漏掉 2 个目标，新增 1 个未匹配框。对同一参考图的自匹配分数为 0.92～0.98，说明模型与视觉提示入口可以运行，但这些参考无法在这批独立场景上稳定迁移。DINO 的 32 个未匹配框是原始候选，并非最终执行目标；包括桌架、夹爪、画面大区域及其他背景。

## Qwen 对 DINO 候选的筛选与误选复核

逐图提交原图、同一套实拍参考和 DINO 的候选编号；没有调用候选之外的粗定位回退。Qwen 在两张正样本里各选对一个目标。4 张负样本中，3 张返回 not_found，1 张把右下角带通孔的黑色夹爪选成电池。单帧筛选耗时 7.25～117.14 秒，包含云端等待，仅作本次运行记录。

对这个误选进一步执行当前 SAM2 分割和 Qwen 第二次复核：
- SAM2 输出掩膜分数 0.984，掩膜约占原图 11.5%。
- Qwen 第二次仍返回 accepted，认为带孔夹爪满足“壳体＋端部把手＋长圆形孔”。
- 该场景人工标注无目标，第二次复核属于误通过。高分割分数无法证明电池身份正确。

本轮没有测量两个模型在同一 GPU 上的延迟，也没有对 YOLOE 进行额外训练。没有评估实际抓取成功率；少量历史图片不足以得出总体准确率。

## 结论

在这批历史样本和新实拍参考条件下，继续使用 DINO 生成候选更有依据。Qwen 筛选能减少部分背景，但夹爪混淆仍会连续误通过，当前复核状态只能用于只读识别和候选展示。下一步修正应围绕整体壳体比例、把手与壳体结构关系，以及已知机器人夹爪区域的排除，再用本次失败帧回归。

## ROG 复现

工作区 /home/arx/arx-vision-demo，分支 codex/vision-demo。

- vision/demo/compare_detectors.py dino，通过 vision/demo/.venv/bin/python 运行。
- vision/demo/compare_detectors.py yoloe，通过 /home/arx/arx-competition-control/vision/.venv-detector/bin/python 运行。
- 可给 yoloe 加 --reference d435、--reference d405 或 --reference astra，分别复测单参考。
- vision/demo/compare_detectors.py qwen，读取已保存的 DINO 候选，重跑结构化目标选择。
- vision/demo/compare_detectors.py qwen-mask，重跑负样本 1 的分割与第二次复核。

原图清单、人工框与 SHA256 在 vision/demo/runtime/detector-compare/dataset.json。所有逐帧输出、Qwen 判定和叠加图保存在同目录，运行时产物未纳入 Git。comparison.jpg 是主配置的左右对照图；false-positive-mask.jpg 展示误通过的夹爪掩膜。

## 输入校验

- 0 — /home/arx/arx-vision-demo/vision/demo/runtime/run-20261108T133344Z-29e60f/observation-1/rgb.png — 5faab98698cf47c5e6be0eca0a8159cf48f6ac2ce6205d7347b7d70d11ce24fe
- 1 — /home/arx/arx-vision-demo/vision/demo/runtime/run-20261108T133529Z-0d626d/observation-1/rgb.png — 97b2ec0316ab2bbbfdead7962b4225d3b6113dcf4ee96e33fcd7fe3824854c6c
- 2 — /home/arx/arx-vision-demo/vision/demo/runtime/run-20261108T133709Z-773c29/observation-1/rgb.png — cd9877232188a4c00019e4a266934b3457133e9e4b25a9e45e279dbdd6064573
- 5 — /home/arx/arx-vision-demo/vision/demo/runtime/run-20261108T133923Z-41e513/observation-1/rgb.png — 153ea241a6f5716a4f59ecdd05fb475e480e11e654fd579dd5d3fef47893441a
- 6 — /home/arx/arx-vision-demo/vision/demo/runtime/run-20261109T095415Z-ad68e6/observation-1/rgb.png — df0649e24a3ead49ec066932ab2612e65c7dc2016a5613a4df989b9891cc3958
- 7 — /home/arx/arx-competition-control/web/hardware/runtime/vision-recognition/c3054b3e5e3d4464b77327cca1dc5f89/review-48/rgb.png — 44f54c160eac031b94483b9872872d54252733a0ddab5e429146403617fe34d3
