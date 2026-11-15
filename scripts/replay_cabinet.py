#!/usr/bin/env python3
"""Replay saved cabinet frames, with no ROS, controller, camera or motion transport."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'web/src')]
from cabinet_preview import CabinetPreview
from cabinet_capture import CALIBRATIONS


class OfflineState:
    models={};config={'arms':{}}
    def snapshot(self):raise RuntimeError('Offline replay cannot read a robot')


def replay(path,model,output):
    path=Path(path);output=Path(output)
    with np.load(path,allow_pickle=False) as saved:frame={k:saved[k].copy() for k in saved.files}
    camera=str(frame['camera_id']);intrinsics=json.loads(str(frame['intrinsics']))
    if camera not in CALIBRATIONS:raise ValueError('未知存档相机身份')
    calibration=(ROOT/CALIBRATIONS[camera]).read_bytes();cal=json.loads(calibration)
    matched=hashlib.sha256(calibration).hexdigest()==str(frame.get('calibration_sha256',''))
    serial=cal['serial'] if camera=='d435' else cal['camera']['serial']
    if matched and intrinsics.get('serial')!=serial:raise ValueError('存档相机与标定序列号不符')
    frame.setdefault('camera_serial',intrinsics.get('serial',''))
    frame.setdefault('calibration_status',cal.get('status','未声明') if matched else '历史外参；与当前标定文件不一致')
    preview=CabinetPreview(OfflineState(),None,captures={},root=ROOT)
    result=preview.analyze_frame(camera,model,frame,now=float(frame['image_time']))
    image=result.pop('image');result['replay']={'historical':True,'input_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'actual_frame_age_s':time.time()-float(frame['image_time']),'clock':'historical exposure time',
        'calibration_matches_current_file':matched,'robot_model_checked':False}
    result['validity']={'valid':False,'reason':'历史离线回放，不能作为当前目标或执行许可'}
    result['executable']=False;result['readiness']['execution_ready']=False
    result['readiness']['blockers'].append('离线回放未连接机器人，未验证当前状态或实机路径')
    output.mkdir(parents=True,exist_ok=True)
    (output/'preview.jpg').write_bytes(base64.b64decode(image.partition(',')[2],validate=True))
    (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frame',type=Path);parser.add_argument('--model',choices=['demo','competition'],default='demo')
    parser.add_argument('--output',required=True,type=Path);args=parser.parse_args()
    try:result=replay(args.frame,args.model,args.output)
    except (ValueError,KeyError,OSError) as e:parser.exit(2,str(e)+'\n')
    print(json.dumps({'status':result['status'],'output':str(args.output.resolve()),'executable':False,
        'fixed_z_range_mm':result.get('plan',{}).get('z_range_mm'),'reason':result.get('reason','')},ensure_ascii=False))


if __name__=='__main__':main()
