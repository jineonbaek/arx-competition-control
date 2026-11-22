"""Reuse computation within one frozen frame only; never cache target decisions."""
import copy
import hashlib
import json
import numpy as np


class FrameModels:
    def __init__(self,models):
        self.models=models
        self.detections={};self.masks={};self.embeddings={}

    @staticmethod
    def _image_key(image):
        return image.mode,image.size,hashlib.sha256(image.tobytes()).digest()

    def detect(self,image,text,threshold=.20):
        key=self._image_key(image),text,float(threshold)
        if key not in self.detections:
            result=(self.models.detect(image,text) if threshold==.20
                    else self.models.detect(image,text,threshold=threshold))
            self.detections[key]=copy.deepcopy(result)
        return copy.deepcopy(self.detections[key])

    def segment(self,image,*,box=None,point=None):
        image_key=self._image_key(image)
        key=image_key,json.dumps(dict(box=box,point=point),allow_nan=False)
        if key not in self.masks:
            sam=getattr(self.models,'sam',None)
            if (callable(getattr(type(sam),'get_image_embeddings',None))
                    and all(hasattr(self.models,k) for k in ('sam_processor','device'))):
                result=self._sam(image,image_key,box=box,point=point)
            else:
                result=self.models.segment(image,box=box,point=point)
            self.masks[key]=np.asarray(result[0],bool).copy(),float(result[1])
        mask,score=self.masks[key]
        return mask.copy(),score

    def _sam(self,image,image_key,*,box,point):
        import torch
        m=self.models
        kwargs={'input_boxes':[[box]]} if box is not None else {'input_points':[[[point]]],'input_labels':[[[1]]]}
        with torch.inference_mode():
            inputs=m.sam_processor(images=image,return_tensors='pt',**kwargs).to(m.device)
            pixels=inputs.pop('pixel_values')
            if image_key not in self.embeddings:
                self.embeddings[image_key]=m.sam.get_image_embeddings(pixels)
            output=m.sam(image_embeddings=self.embeddings[image_key],**inputs)
            masks=m.sam_processor.post_process_masks(output.pred_masks.cpu(),inputs['original_sizes'].cpu())[0]
            scores=output.iou_scores.reshape(-1).cpu();index=int(scores.argmax())
            return masks.reshape(-1,image.height,image.width)[index].numpy().astype(bool),float(scores[index])
