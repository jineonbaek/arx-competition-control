"""Local CUDA inference using pinned snapshots, with original-image coordinates."""
from pathlib import Path
import numpy as np
import torch
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor, Sam2Model, Sam2Processor

ROOT = Path(__file__).resolve().parent

class VisionModels:
    def __init__(self, device="cuda"):
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA unavailable")
        self.device = device
        torch.set_num_threads(4)
        self.dino_processor = AutoProcessor.from_pretrained(ROOT/"models/dino", local_files_only=True)
        self.dino = AutoModelForZeroShotObjectDetection.from_pretrained(
            ROOT/"models/dino", local_files_only=True).to(device).eval()
        self.sam_processor = Sam2Processor.from_pretrained(ROOT/"models/sam2", local_files_only=True)
        self.sam = Sam2Model.from_pretrained(ROOT/"models/sam2", local_files_only=True).to(device).eval()

    @torch.inference_mode()
    def detect(self, image, text, threshold=.20):
        inputs = self.dino_processor(images=image, text=text, return_tensors="pt").to(self.device)
        outputs = self.dino(**inputs)
        result = self.dino_processor.post_process_grounded_object_detection(
            outputs, inputs.input_ids, threshold=threshold, text_threshold=.20,
            target_sizes=[(image.height, image.width)])[0]
        labels = result.get("text_labels", result.get("labels"))
        rows = []
        for box, score, label in zip(result["boxes"], result["scores"], labels):
            x1,y1,x2,y2 = box.cpu().tolist()
            box = [max(0,int(x1)),max(0,int(y1)),min(image.width,int(x2)),min(image.height,int(y2))]
            if box[2]-box[0]>=8 and box[3]-box[1]>=8:
                rows.append({"bbox_xyxy":box,"score":float(score),"label":str(label)})
        rows.sort(key=lambda row: row["score"], reverse=True)
        return [dict(row,candidate_id=i+1) for i,row in enumerate(rows[:12])]

    @torch.inference_mode()
    def segment(self, image, *, box=None, point=None):
        kwargs = {"input_boxes":[[box]]} if box is not None else {
            "input_points":[[[point]]], "input_labels":[[[1]]]}
        inputs = self.sam_processor(images=image, return_tensors="pt", **kwargs).to(self.device)
        outputs = self.sam(**inputs)
        masks = self.sam_processor.post_process_masks(
            outputs.pred_masks.cpu(), inputs["original_sizes"].cpu())[0]
        scores = outputs.iou_scores.reshape(-1).cpu()
        index = int(scores.argmax())
        mask = masks.reshape(-1,image.height,image.width)[index].numpy().astype(bool)
        return mask, float(scores[index])
