import unittest
import numpy as np
from PIL import Image
from agent.frame_vision import FrameModels


class CountingModels:
    def __init__(self):self.detect_calls=0;self.segment_calls=0
    def detect(self,image,text,threshold=.20):
        self.detect_calls+=1
        return [dict(candidate_id=1,bbox_xyxy=[1,1,3,3],score=.8,label=text)]
    def segment(self,image,*,box=None,point=None):
        self.segment_calls+=1
        mask=np.zeros((image.height,image.width),bool)
        if box:mask[box[1]:box[3],box[0]:box[2]]=True
        else:mask[point[1],point[0]]=True
        return mask,.9


class FrameVisionTests(unittest.TestCase):
    def test_identical_frame_prompt_and_mask_are_computed_once_and_copied(self):
        base=CountingModels();v=FrameModels(base);im=Image.new('RGB',(8,8))
        a=v.detect(im,'handle');a[0]['bbox_xyxy'][0]=99
        b=v.detect(im.copy(),'handle')
        self.assertEqual(b[0]['bbox_xyxy'],[1,1,3,3]);self.assertEqual(base.detect_calls,1)
        mask,score=v.segment(im,box=[1,1,3,3]);mask[:]=False
        again,_=v.segment(im.copy(),box=[1,1,3,3])
        self.assertEqual(int(again.sum()),4);self.assertEqual(base.segment_calls,1)

    def test_new_frame_prompt_threshold_and_box_do_not_reuse_old_answers(self):
        base=CountingModels();v=FrameModels(base);im=Image.new('RGB',(8,8))
        v.detect(im,'handle');v.detect(im,'battery');v.detect(im,'handle',.3)
        changed=im.copy();changed.putpixel((0,0),(1,2,3));v.detect(changed,'handle')
        FrameModels(base).detect(im,'handle')
        self.assertEqual(base.detect_calls,5)
        v.segment(im,box=[1,1,3,3]);v.segment(im,box=[2,2,4,4]);v.segment(changed,box=[1,1,3,3])
        FrameModels(base).segment(im,box=[1,1,3,3])
        self.assertEqual(base.segment_calls,4)


if __name__=='__main__':unittest.main()
