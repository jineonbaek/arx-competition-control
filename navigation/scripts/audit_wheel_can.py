#!/usr/bin/env python3
"""Passive CAN ID/feedback audit; never calls send/sendto/write/ioctl transmit."""
from pathlib import Path
import argparse
import collections
import socket
import struct
import time
import json
import csv
import signal

p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=15);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW);s.bind(('can5',));s.settimeout(.1);end=time.monotonic()+a.duration;counts=collections.Counter();local=collections.Counter();wheel=[]
started=time.monotonic()
def stop(signum,frame):
    global end
    end=time.monotonic()
signal.signal(signal.SIGINT,stop)
while time.monotonic()<end:
    try:data,anc,flags,addr=s.recvmsg(16)
    except socket.timeout:continue
    canid,dlc,payload=struct.unpack('=IB3x8s',data)
    if flags&socket.MSG_DONTROUTE:local[hex(canid)]+=1;continue
    counts[hex(canid)]+=1
    if canid==0x702 and dlc==8:wheel.append([time.time(),*struct.unpack('>hhhh',payload)])
s.close();result={'duration_s':time.monotonic()-started,'requested_duration_s':a.duration,'external_rx_ids':dict(counts),'local_origin_ids':dict(local),'wheel_feedback_0x702_frames':len(wheel),'first_last_wheel_payload':wheel[:1]+wheel[-1:] if wheel else [],'transmitted_frames':0};a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

with a.output.with_suffix('.csv').open('w',newline='') as f:
    writer=csv.writer(f);writer.writerow(['receive_epoch_s','raw_0','raw_1','raw_2','raw_3','sdk_0','sdk_1','sdk_2','sdk_3'])
    for row in wheel:writer.writerow(row+[v*10/32768 for v in row[1:]])
