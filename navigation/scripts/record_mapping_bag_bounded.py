#!/usr/bin/env python3
"""Bounded local raw-sensor recording; no robot command publishers."""
import argparse,os,signal,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=1800);p.add_argument('--topic-prefix',default='/mapping_test');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
cmd=['ros2','bag','record','-s','mcap','--max-bag-size','536870912','--max-cache-size','104857600','-o',str(a.output/'raw-bag'),a.topic_prefix+'/lidar',a.topic_prefix+'/imu',a.topic_prefix+'/odometry','/tf','/tf_static']
with (a.output/'bag-recorder.log').open('w') as f:
 child=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);(a.output/'bag-recorder.pid').write_text(str(child.pid)+'\n')
 try:child.wait(timeout=a.duration)
 except subprocess.TimeoutExpired:
  os.killpg(child.pid,signal.SIGINT)
  try:child.wait(timeout=20)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
