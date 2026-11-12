"""Shared target for the live overlay, sampled Qwen review, and offline pipeline."""
from pathlib import Path
from PIL import Image

REFERENCE = Path(__file__).resolve().parents[1] / "config/battery-reference.png"
DINO_PROMPT = "a removable drone battery pack. a rectangular battery case with a side latch and end handle. a battery mounted on a drone."
TASK = ("找到无人机上的可拆卸比赛电池。比赛起始区域通常只有无人机和电池。"
        "按整块电池的可见外形判断：长方形或圆角矩形外壳、侧面卡扣、端部手柄和电源连接位置。"
        "不要求看到长圆孔、四个小圆孔或其他孔位；这些细节可能被遮挡，定位和抓取由 D405 与 FoundationPose 完成。"
        "左下角和右下角的固定夹爪是机器人自身，必须排除；仓口、金属筐、桌面和无人机其他结构属于背景。"
        "忽略颜色、明暗、材质、胶带、条码和文字差异，打印件可以没有标签。"
        "若当前视角只能看到电池壳体而看不到孔位，仍应按整体外形选择最接近的电池；只有确实没有电池候选时才返回 not_found。")


def shape_view(image):
    return image.convert("L").convert("RGB")


def load_reference():
    with Image.open(REFERENCE) as image:
        return image.convert("RGB")
