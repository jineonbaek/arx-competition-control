"""Read timestamp metadata from Ethernet pcap, no scene payload export."""
import argparse,struct,json,collections
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('pcap');p.add_argument('--output',required=True);a=p.parse_args()
f=open(a.pcap,'rb');h=f.read(24);endian='<' if h[:4] in (b'\xd4\xc3\xb2\xa1',b'\x4d\x3c\xb2\xa1') else '>';streams=collections.defaultdict(list)
while True:
 h=f.read(16)
 if len(h)!=16:break
 sec,us,n,orig=struct.unpack(endian+'4I',h);b=f.read(n)
 if len(b)<78 or b[12:14]!=b'\x08\x00':continue
 off=14+(b[14]&15)*4;ip='.'.join(map(str,b[26:30]));d=b[off+8:]
 if len(d)<36:continue
 port=struct.unpack('!H',b[off+2:off+4])[0]
 typ=d[11];dtype=d[10];ts=struct.unpack('<Q',d[28:36])[0]
 streams[f'{ip}:{port}:{dtype}'].append((sec*10**9+us*1000,ts,typ))
r={'streams':{},'ptp_type_all':bool(streams),'note':'Receive-minus-device includes transport and scheduling delay; not an absolute clock accuracy measurement.'}
for k,v in streams.items():
 arr=np.array(v,dtype=np.int64);delay=(arr[:,0]-arr[:,1])/1e6;delta=np.diff(arr[:,1]);types=dict(collections.Counter(str(x[2]) for x in v))
 r['ptp_type_all'] &= set(types)=={'1'}
 r['streams'][k]={'count':len(v),'types':types,'nonincreasing':int(np.sum(delta<=0)),'device_gap_max_ms':float(delta.max()/1e6) if len(delta) else None,'receive_minus_device_ms_percentiles':np.percentile(delay,[0,50,95,99,100]).tolist(),'delay_first_half_median_ms':float(np.median(delay[:len(delay)//2])),'delay_last_half_median_ms':float(np.median(delay[len(delay)//2:]))}
Path(a.output).write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
