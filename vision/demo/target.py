"""Shared target for the live overlay, sampled Qwen review, and offline pipeline."""
from pathlib import Path
from PIL import Image

REFERENCE = Path(__file__).resolve().parents[1] / "config/battery-reference.png"
DINO_PROMPT = "drone battery handle . battery carry handle . removable drone battery ."
TASK = ("找到安装在无人机电池槽中的可拆卸比赛电池。\n\n"
        "当前D435视角通常只能看到电池外露端部和前端提手，长方体电池主体大部分被无人机机身遮挡。"
        "因此不要要求看到完整电池外壳、侧面卡扣、电源连接器、标签或其他孔位。\n\n"
        "主要依据电池前端提手判断：提手与电池外端面连成一体，具有宽横向、圆角矩形的提手开口或提手槽，"
        "提手后方连接被无人机机身遮挡的长方体电池主体。目标位于无人机电池槽口处，而不是地面或无人机机架上。\n\n"
        "优先选择同时包含提手和相连电池端面的候选。若候选只覆盖提手，但其位置明确位于电池槽口并与电池端面连接，也可以选择。\n\n"
        "必须排除无人机起落架、机臂、螺旋桨、空电池槽、地面标定板、桌面，以及画面下方的机器人夹爪和机械臂。"
        "忽略颜色、明暗、材质、标签、条码和打印纹理差异。\n\n"
        "如果多个候选无法可靠区分，返回 ambiguous；如果画面中确实没有电池提手或电池外端部，返回 not_found。")


def shape_view(image):
    return image.convert("L").convert("RGB")


def load_reference():
    with Image.open(REFERENCE) as image:
        return image.convert("RGB")
