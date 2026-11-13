"""Passive PTP/HMS gate. No commands or time changes are sent to the lidar."""
import collections,json,socket,struct,sys
from pathlib import Path

def packets(path):
 with open(path,'rb') as f:
  header=f.read(24)
  if len(header)!=24:raise ValueError('Incomplete pcap')
  le=header[:4] in (b'\xd4\xc3\xb2\xa1',b'\x4d\x3c\xb2\xa1');fmt='<' if le else '>'
  while h:=f.read(16):
   if len(h)!=16:raise ValueError('Incomplete record')
   _,_,n,_=struct.unpack(fmt+'IIII',h);p=f.read(n)
   if len(p)<34:continue
   et=int.from_bytes(p[12:14],'big')
   if et==0x88f7:yield 'L2',0,p[14:];continue
   if et!=0x800:continue
   ip=p[14:];hl=(ip[0]&15)*4
   if ip[9]!=17 or len(ip)<hl+8:continue
   dp=int.from_bytes(ip[hl+2:hl+4],'big')
   yield socket.inet_ntoa(ip[12:16]),dp,ip[hl+8:]

def audit(path,flags):
 counts=collections.Counter();masters=collections.Counter();sync=set();follow=set();responses=collections.Counter();requests=set();answers=set();hms=collections.defaultdict(list);mac={};l2servers=set()
 for src,port,p in packets(path):
  if port in (319,320) and len(p)>=34:
   kind=p[0]&15;counts[str(kind)]+=1;identity=p[20:28].hex();sequence=int.from_bytes(p[30:32],'big')
   if kind in (0,8,11):masters[identity]+=1
   if kind==1:requests.add((identity,sequence))
   if kind==0:sync.add((identity,sequence))
   if kind==8:follow.add((identity,sequence))
   if kind==9 and len(p)>=54:answers.add((p[44:52].hex(),sequence))
  elif src=='L2' and len(p)>=34 and (p[0]&15) in (0,8,11):l2servers.add(p[20:28].hex())
  elif port==56201 and src in ('192.168.1.3','192.168.1.164'):
   tag=struct.pack('<HH',0x8011,32);i=p.find(tag)
   if i>=0 and len(p)>=i+36:hms[src].append(list(struct.unpack('<8I',p[i+4:i+36])))
   i=p.find(struct.pack('<HH',0x8005,6))
   if i>=0 and len(p)>=i+10:
    a=p[i+4:i+10];mac[src]=(a[:3]+b'\xff\xfe'+a[3:]).hex()
 for identity,sequence in requests&answers:responses[identity]+=1
 errors=[]
 if len(masters)!=1:errors.append('Expected exactly one UDP PTP master')
 if len(sync&follow)<8:errors.append('Fewer than eight matched Sync/Follow_Up pairs')
 if l2servers:errors.append('Concurrent L2/gPTP master observed')
 required=['192.168.1.3']+(['192.168.1.164'] if flags['head'] else [])
 for ip in required:
  if len(hms[ip])<3:errors.append(ip+': insufficient status samples')
  if responses[mac.get(ip,'')]<3:errors.append(ip+': no matching Delay_Resp sequence')
  for frame in hms[ip]:
   if any((v>>16) in (0x0402,0x0403,0x0404,0x0406,0x0407,0x0408,0x0409,0x040a) for v in frame):
    errors.append(ip+': current synchronization warning');break
 return dict(acceptable=not errors,errors=errors,ptp_message_counts=dict(counts),matched_sync_followup=len(sync&follow),master_identities=dict(masters),delay_responses=dict(responses),hms={k:[[hex(x) for x in f if x] for f in a] for k,a in hms.items()},note='Protocol/status gate only; software timestamp precision is not a hardware accuracy guarantee')
if __name__=='__main__':
 flags=json.loads(Path(sys.argv[2]).read_text());r=audit(sys.argv[1],flags);Path(sys.argv[3]).write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));sys.exit(0 if r['acceptable'] else 1)
