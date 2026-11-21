"""Reviewed D405 frames and battery-only crops for Qwen reference evidence."""
from pathlib import Path
from PIL import Image

REFERENCES = {
    'd405-01': {'label':'图1 · 完整成功 · 首次远景', 'box':[266,46,350,137]},
    'd405-02': {'label':'图2 · 完整成功 · 接近近景', 'box':[245,57,457,353]},
    'd405-03': {'label':'图3 · 夹持确认 · 首次远景', 'box':[329,29,435,132]},
    'd405-04': {'label':'图4 · 夹持确认 · 接近近景', 'box':[232,68,444,363]},
}


def reference_image(identity):
    if identity not in REFERENCES:raise ValueError('Unknown D405 reference image')
    path=Path(__file__).resolve().parents[1]/'web/public/grasp-reference'/(identity+'.png')
    with Image.open(path) as image:return image.crop(REFERENCES[identity]['box']).convert('RGB')
