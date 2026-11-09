"""Compare saved DINO validation scenes with the deployed YOLOE and DINO inputs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np
from PIL import Image,ImageDraw

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'runtime/detector-compare'


def overlap(a,b):
    size=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    return size/max(1,(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-size)


def evaluate(rows,truth):
    used=set();hits=0
    for row in sorted(rows,key=lambda r:-r['score']):
        candidates=[(overlap(row['bbox_xyxy'],b),j) for j,b in enumerate(truth) if j not in used]
        if candidates and max(candidates)[0]>=.5:
            used.add(max(candidates)[1]);hits+=1
    return dict(tp=hits,fp=len(rows)-hits,fn=len(truth)-hits)


def review():
    from concurrent.futures import ThreadPoolExecutor,as_completed
    from pipeline import QwenJudge,validate_choice
    from target import load_reference,shape_view,TASK
    data=json.loads((OUT/'dataset.json').read_text())
    pred=json.loads((OUT/'dino.json').read_text())['frames']
    def judge(row):
        cand=next(r['candidates'] for r in pred if r['index']==row['index'])
        im=shape_view(Image.open(row['path']));ref=shape_view(load_reference())
        client=QwenJudge(Path('/home/arx/arx-competition-control/agent/.secrets/qwen.env'))
        start=time.monotonic()
        try:
            result=client.select(im,ref,cand,TASK)
            choice=validate_choice(result,cand,im.size,False)
            return dict(index=row['index'],selected=result,box=choice.get('box') if choice else None,
                        seconds=round(time.monotonic()-start,2))
        except Exception as e:
            return dict(index=row['index'],error=type(e).__name__,seconds=round(time.monotonic()-start,2))
    reports=[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        for future in as_completed([pool.submit(judge,row) for row in data]):
            reports.append(future.result())
            (OUT/'qwen-selection.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2))
            print(reports[-1],flush=True)


def review_mask():
    from models import VisionModels
    from pipeline import QwenJudge,mask_geometry,overlay_mask
    from target import shape_view,load_reference,TASK
    data=json.loads((OUT/'dataset.json').read_text())
    # This independently labeled negative scene was selected as a target by Qwen.
    chosen=next(r for r in json.loads((OUT/'qwen-selection.json').read_text()) if r['index']==1)
    image=Image.open(next(r['path'] for r in data if r['index']==1)).convert('RGB')
    models=VisionModels()
    mask,score=models.segment(shape_view(image),box=chosen['box'])
    overlay_mask(image,mask).save(OUT/'false-positive-mask.jpg')
    judge=QwenJudge(Path('/home/arx/arx-competition-control/agent/.secrets/qwen.env'))
    start=time.monotonic()
    result=judge.review(shape_view(image),shape_view(load_reference()),
                        overlay_mask(shape_view(image),mask),mask_geometry(mask),TASK)
    report=dict(index=1,review=result,seconds=round(time.monotonic()-start,2),
                mask_fraction=float(mask.mean()),sam2_score=score)
    (OUT/'qwen-false-positive-review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))


def main():
    p=argparse.ArgumentParser()
    p.add_argument('model',choices=['dino','yoloe','qwen','qwen-mask'])
    p.add_argument('--reference',choices=['d435','d405','astra'])
    args=p.parse_args()
    if args.model=='qwen-mask':
        review_mask()
        return
    if args.model=='qwen':
        review()
        return
    data=json.loads((OUT/'dataset.json').read_text())
    for r in data:assert hashlib.sha256(Path(r['path']).read_bytes()).hexdigest()==r['sha256']
    if args.model=='dino':
        from models import VisionModels
        from target import shape_view,DINO_PROMPT
        import torch
        model=VisionModels()
        def detect(im):return model.detect(shape_view(im),DINO_PROMPT,.20)
        device='ROG CUDA GPU'
        sync=lambda:torch.cuda.synchronize()
    else:
        os.environ['YOLO_AUTOINSTALL']='false'
        from ultralytics import YOLOE,settings
        import torch
        settings.update({'sync':False});torch.set_num_threads(4)
        ref=ROOT.parent/'config/battery-reference.png'
        prompts=json.loads(ref.with_suffix('.json').read_text())['examples']
        if args.reference:
            cfg=json.loads((ref.parent/'whole-battery-references.json').read_text())['references'][args.reference]
            ref=ref.parent/cfg['image']
            prompts=[dict(bbox=b,**{'class':c}) for b,c in zip(cfg['boxes'],cfg['classes'])]
        arr=np.asarray(Image.open(ref).convert('L').convert('RGB'))
        visual={'bboxes':np.array([r['bbox'] for r in prompts]),'cls':np.array([r['class'] for r in prompts])}
        model=YOLOE('/home/arx/arx-competition-control/vision/models/yoloe-11s-seg.pt')
        model.predict(arr,refer_image=arr,visual_prompts=visual,device='cpu',imgsz=640,verbose=False)
        def detect(im):
            result=model.predict(np.asarray(im.convert('L').convert('RGB')),device='cpu',imgsz=640,conf=.01,iou=.5,verbose=False)[0]
            rows=[dict(bbox_xyxy=b.xyxy[0].tolist(),score=float(b.conf[0])) for b in result.boxes if int(b.cls[0])==0]
            return [dict(r,candidate_id=i+1) for i,r in enumerate(sorted(rows,key=lambda r:-r['score'])[:12])]
        device='ROG CPU; 4 threads'
        sync=lambda:None
    detect(Image.open(data[0]['path']).convert('RGB'));sync()
    report=[]
    for row in data:
        im=Image.open(row['path']).convert('RGB');times=[]
        for _ in range(3):
            sync()
            start=time.perf_counter()
            boxes=detect(im)
            sync()
            times.append((time.perf_counter()-start)*1000)
        below=boxes
        boxes=[r for r in boxes if r['score']>=.20]
        annotated=im.copy();draw=ImageDraw.Draw(annotated)
        for b in row['boxes']:draw.rectangle(b,outline='yellow',width=2)
        for r in boxes:
            b=r['bbox_xyxy'];draw.rectangle(b,outline='cyan',width=2);draw.text((b[0],max(0,b[1]-12)),str(r['candidate_id'])+' '+format(r['score'],'.2f'),fill='red')
        annotated.save(OUT/(args.model+'-'+str(row['index'])+'.jpg'))
        report.append(dict(index=row['index'],truth=row['boxes'],candidates=boxes,median_ms=float(np.median(times)),low_threshold_candidates=below,**evaluate(boxes,row['boxes'])))
    result=dict(model=args.model,device=device,threshold=.20,matching_iou=.5,frames=report)
    (OUT/(args.model+('-'+args.reference if args.reference else '')+'.json')).write_text(json.dumps(result,indent=2))
    print(args.model,[(r['index'],r['tp'],r['fp'],r['fn'],round(r['median_ms'])) for r in report],flush=True)


if __name__=='__main__':main()
