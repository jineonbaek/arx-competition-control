"""Verify both deployed checkpoints on the actual GPU without opening a camera."""
import importlib.metadata
import json
from pathlib import Path
import time
import torch
from PIL import Image
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor, Sam2Model, Sam2Processor

ROOT = Path(__file__).resolve().parent

def main():
    assert torch.cuda.is_available(), "CUDA unavailable"
    torch.set_num_threads(4)
    torch.cuda.reset_peak_memory_stats()
    started = time.monotonic()
    image = Image.new("RGB", (640, 480), (60, 60, 60))
    dino = AutoModelForZeroShotObjectDetection.from_pretrained(ROOT / "models/dino", local_files_only=True).to("cuda").eval()
    dp = AutoProcessor.from_pretrained(ROOT / "models/dino", local_files_only=True)
    sam = Sam2Model.from_pretrained(ROOT / "models/sam2", local_files_only=True).to("cuda").eval()
    sp = Sam2Processor.from_pretrained(ROOT / "models/sam2", local_files_only=True)
    with torch.inference_mode():
        di = dp(images=image, text="a white plastic object.", return_tensors="pt").to("cuda")
        dout = dino(**di)
        si = sp(images=image, input_boxes=[[[100, 100, 300, 300]]], return_tensors="pt").to("cuda")
        sout = sam(**si)
    assert torch.isfinite(dout.logits).all() and torch.isfinite(sout.pred_masks).all()
    print(json.dumps({"gpu": torch.cuda.get_device_name(), "dino_logits": list(dout.logits.shape),
                      "sam_masks": list(sout.pred_masks.shape),
                      "peak_gpu_mib": round(torch.cuda.max_memory_allocated()/2**20),
                      "seconds": round(time.monotonic()-started, 2),
                      "packages": {p: importlib.metadata.version(p) for p in
                                   ["torch", "torchvision", "transformers", "huggingface-hub"]}}), flush=True)

if __name__ == "__main__":
    main()
