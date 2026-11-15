#!/usr/bin/env python3
"""Replay held-battery/docking evidence offline; no ROS, network or actuator transport."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from agent.cabinet_replay import run_demo,replay_recorded


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--demo',action='store_true');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--d435-frame',type=Path);p.add_argument('--d405-image',type=Path);p.add_argument('--d405-meta',type=Path);p.add_argument('--d405-reference',type=Path)
    a=p.parse_args()
    if a.demo and any((a.d435_frame,a.d405_image,a.d405_meta,a.d405_reference)):p.error('Synthetic demo and recorded evidence must be separate')
    if not a.demo and not any((a.d435_frame,a.d405_image)):p.error('Provide --demo or recorded frame/image files')
    try:
        r=run_demo(a.output) if a.demo else replay_recorded(a.output,d435_frame=a.d435_frame,d405_image=a.d405_image,d405_meta=a.d405_meta,d405_reference=a.d405_reference)
    except (ValueError,OSError,KeyError) as e:p.exit(2,str(e)+'\n')
    print(json.dumps(dict(mode='offline',executable=False,output=str(a.output.resolve()),
        scenarios={k:v['state']['status'] for k,v in r.get('scenarios',{}).items()},historical=r.get('historical',False)),ensure_ascii=False))


if __name__=='__main__':main()
