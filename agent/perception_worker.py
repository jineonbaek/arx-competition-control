"""Task-scoped local recognition and pose refinement over immutable RGB-D files."""
import json
from pathlib import Path
import sys
import time
import traceback
from types import SimpleNamespace
import numpy as np
from PIL import Image
from .storage import atomic_json
from .grasp_geometry import rigid,validate_frame

ROOT=Path(__file__).resolve().parents[1]
VISION=Path('/home/arx/arx-vision-demo/vision/demo')


def box_iou(a,b):
    a=np.asarray(a);b=np.asarray(b)
    area=np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2])).prod()
    return float(area/max(1,(a[2:]-a[:2]).prod()+(b[2:]-b[:2]).prod()-area))


def local_choice(candidates,frame,models,image,target_mm=None):
    """Accept a distinct local candidate, otherwise leave semantic selection to Qwen."""
    from .battery_observation import surface_location
    from target import shape_view
    ranked=[]
    for row in candidates:
        if row['score']<.30:continue
        mask,score=models.segment(shape_view(image),box=row['bbox_xyxy'])
        if score<.85:continue
        try:location=surface_location(frame,mask)
        except ValueError:continue
        surface=np.asarray(location['surface_base_mm'],float)
        distance=(float(np.linalg.norm(surface-target_mm))
                  if target_mm is not None else 0.)
        if target_mm is not None and distance>120:continue
        ranked.append((distance,row,mask,score,surface))
    unique=[]
    # A nested mask at the same surface is one target; retain its most complete mask.
    for item in sorted(ranked,key=lambda r:(int(r[2].sum()),r[1]['score']),reverse=True):
        if any(box_iou(item[1]['bbox_xyxy'],old[1]['bbox_xyxy'])>0
               and np.logical_and(item[2],old[2]).sum()>=.9*min(item[2].sum(),old[2].sum())
               and np.linalg.norm(item[4]-old[4])<40 for old in unique):continue
        unique.append(item)
    ranked=unique
    if target_mm is not None:
        ranked.sort(key=lambda row:row[0])
        if len(ranked)>1 and ranked[1][0]-ranked[0][0]<40:return None
    else:
        ranked.sort(key=lambda row:row[1]['score'],reverse=True)
        if len(ranked)>1 and ranked[0][1]['score']-ranked[1][1]['score']<.10:return None
    return ranked[0][1:4] if ranked else None


def projected_box(frame,state):
    from web.src.camera_geometry import camera_intrinsics
    base=validate_frame(frame)
    pose=np.linalg.inv(base)@rigid(state['base_from_object'])
    points=np.asarray(state['mesh'].vertices)@pose[:3,:3].T+pose[:3,3]
    if np.any(points[:,2]<=.07):raise ValueError('Tracked battery behind camera')
    k=json.loads(str(frame['intrinsics']));rs,intr=camera_intrinsics(k)
    pixels=np.array([rs.rs2_project_point_to_pixel(intr,p.tolist()) for p in points])
    low=pixels.min(0);high=pixels.max(0);margin=np.maximum(4,(high-low)*.04)
    box=np.r_[np.maximum(0,low-margin),np.minimum([k['width'],k['height']],high+margin)]
    if not np.isfinite(box).all() or np.any(box[2:]-box[:2]<12):raise ValueError('Tracked battery outside image')
    return box.astype(int).tolist()


class Models:
    def __init__(self):
        self.vision=None;self.pose={};self.head_target=None

    def segment(self,args):
        if str(VISION) not in sys.path:sys.path.insert(0,str(VISION))
        from models import VisionModels
        from pipeline import (QwenJudge,run_frame,overlay_mask,
                              filter_fixed_gripper_candidates,mask_geometry)
        from target import shape_view,load_reference,DINO_PROMPT,TASK
        if self.vision is None:self.vision=VisionModels()
        started=time.monotonic();output=Path(args['output']);output.mkdir(exist_ok=True,parents=True)
        reference=load_reference()
        with np.load(args['frame'],allow_pickle=False) as archive:
            frame={k:archive[k] for k in archive.files}
        validate_frame(frame);camera=str(frame['camera_id']);image=Image.open(args['image']).convert('RGB')
        chosen=None;method='local_candidate';tracking_error=None
        if camera=='d405' and 'base_from_object' in self.pose:
            try:
                box=projected_box(frame,self.pose)
                mask,score=self.vision.segment(shape_view(image),box=box)
                from .battery_observation import surface_location
                surface_location(frame,mask)
                if score<.85:raise ValueError('Tracked mask confidence low')
                y,x=np.nonzero(mask)
                if (x.min()<box[0] or x.max()>=box[2] or y.min()<box[1] or y.max()>=box[3]):
                    raise ValueError('Tracked mask left predicted region')
                chosen=({'candidate_id':0,'bbox_xyxy':box},mask,score);method='pose_tracking'
            except ValueError as error:tracking_error=str(error)
        candidates=[]
        if chosen is None:
            detected=self.vision.detect(shape_view(image),DINO_PROMPT)
            candidates,_=filter_fixed_gripper_candidates(detected,image.size,camera)
            chosen=local_choice(candidates,frame,self.vision,image,
                self.head_target if camera=='d405' else None)
        if chosen is None:
            judge=QwenJudge(ROOT/'agent/.secrets/qwen.env')
            report=run_frame(self.vision,judge,image,reference,TASK,output,camera=camera)
            atomic_json(output/'qwen-reports.json',judge.records)
            report['path']='qwen_fallback'
        else:
            row,mask,score=chosen
            Image.fromarray(mask.astype(np.uint8)*255).save(output/'mask.png')
            overlay_mask(image,mask).save(output/'overlay.png')
            report=dict(status='perception_accepted',path=method,
                selection=dict(status='selected',candidate_id=row['candidate_id'],reason=method),
                review=dict(status='accepted',reason='Local mask, depth and target association'),
                geometry=mask_geometry(mask),sam2_predicted_iou=score,candidates=candidates)
        if report.get('status')=='perception_accepted' and camera=='d435':
            from .battery_observation import surface_location
            mask=np.asarray(Image.open(output/'mask.png'))>0
            self.head_target=np.asarray(surface_location(frame,mask)['surface_base_mm'])
        report.update(output=str(output),elapsed_s=round(time.monotonic()-started,3),tracking_error=tracking_error)
        image.save(output/'rgb.png');atomic_json(output/'recognition.json',report)
        return report

    def estimate(self,args):
        sys.path.insert(0,str(ROOT/'foundationpose'))
        from foundationpose.infer_snapshot import infer
        values={k:(Path(v) if k in ('frame','output','mask','head_frame','head_mask') and v else v)
                for k,v in args.items()}
        values.update(mesh=ROOT/'web/cad/gripper-v2/battery-print.stl',mesh_scale=.001)
        report=infer(SimpleNamespace(**values),self.pose)
        if report['quality']['status']!='consistent' and report['estimation_method']=='track':
            self.pose.pop('base_from_object',None)
            report=infer(SimpleNamespace(**values),self.pose)
            report['estimation_method']='register_recovery'
            atomic_json(values['output'],report)
        return {'output':str(values['output']),'estimation_method':report['estimation_method']}


def main():
    directory=Path(sys.argv[1]).resolve();run=directory.parent
    models=Models();last=None
    while True:
        request=directory/'request.json'
        if not request.exists():time.sleep(.02);continue
        message=json.loads(request.read_text());identity=message['id']
        if not isinstance(identity,str) or len(identity)!=32 or any(c not in '0123456789abcdef' for c in identity):
            raise ValueError('Invalid request ID')
        if identity==last:time.sleep(.02);continue
        last=identity
        try:
            args=message['arguments']
            for key in ('frame','image','output','mask','head_frame','head_mask'):
                if args.get(key) and not Path(args[key]).resolve().is_relative_to(run):raise ValueError('Perception path outside task')
            operation={'segment':models.segment,'estimate':models.estimate}[message['operation']]
            data=operation(args)
            reply=dict(id=identity,ok=True,data=data)
        except Exception as error:
            traceback.print_exc();reply=dict(id=identity,ok=False,error=str(error)[:300])
        atomic_json(directory/(identity+'.json'),reply)


if __name__=='__main__':main()
