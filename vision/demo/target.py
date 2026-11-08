"""Shared target for the live overlay, sampled Qwen review, and offline pipeline."""
from pathlib import Path
from PIL import Image

REFERENCE = Path(__file__).resolve().parents[1] / "config/battery-reference.png"
DINO_PROMPT = "a rectangular battery pack. a removable battery pack with a protruding end handle. a battery case with an oblong handle opening."
TASK = ("找到与参考图外形一致的比赛电池或等尺寸打印件。参考图并排展示同一种电池的三个局部视角。"
        "仅按可见外形判断：长方形壳体、端部凸出的把手及其长圆形通孔，以及这些部位的相对位置。"
        "目标是整块电池的可见部分；仓口、金属筐、无人机和桌面属于背景。"
        "忽略颜色、明暗、材质、胶带、条码和文字差异，打印件可以没有标签。"
        "若遮挡或视角使形状证据不足，请明确返回不确定；用中文解释可见的形状依据。")


def shape_view(image):
    return image.convert("L").convert("RGB")


def load_reference():
    with Image.open(REFERENCE) as image:
        return image.convert("RGB")
