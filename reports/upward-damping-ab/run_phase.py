from pathlib import Path
import socket,struct,time,json,subprocess,sys,os
D=Path(__file__).parent;target=float(sys.argv[1]);label=sys.argv[2];assert target in (2.,5.)
assert Path('/proc/21646/exe').resolve()==Path('/home/arx/arx-competition-control/lift-upward-damping-candidate/lift_controller_candidate')
s=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW);s.bind(('can5',));s.settimeout(.5)
def param(k,v):
 p=subprocess.run(['bash','-c','source /home/arx/LIFT/body/ROS2/install/setup.bash; export ROS_DOMAIN_ID=191 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; ros2 param set /lift_controller '+k+' '+str(v)],capture_output=True,text=True,timeout=10)
 assert p.returncode==0 and 'successful' in p.stdout.lower(),p.stdout+p.stderr
 return p.stdout
start=time.time();result=param('fixed_body_height',target);print(label,result.strip(),flush=True);rows=[];end=time.monotonic()+12
while time.monotonic()<end:
 try:f=s.recv(16)
 except TimeoutError:continue
 cid,n,b=struct.unpack('=IB3x8s',f)
 if cid==0 and n==8 and b[0]&15==1:
  row={'time':time.time(),'height_rad':-(int.from_bytes(b[1:3],'big')*25/65535-12.5),'velocity_rad_s':((b[3]<<4)|(b[4]>>4))*90/4095-45,'state':b[0]>>4,'motor_C':b[7]};rows.append(row)
s.close();recent=[x for x in rows if x['time']>time.time()-2]
summary={'label':label,'command_time':start,'finished_time':time.time(),'target_rad':target,'sample_count':len(rows),'last':rows[-1] if rows else None,'last_2s_span_rad':max(x['height_rad'] for x in recent)-min(x['height_rad'] for x in recent) if recent else None,'all_enabled_feedback':all(x['state']==1 for x in rows)}
(D/(label+'.json')).write_text(json.dumps(dict(summary,samples=rows),indent=2));print(json.dumps(summary),flush=True)
assert len(recent)>100 and summary['all_enabled_feedback'] and summary['last_2s_span_rad']<.005 and abs(recent[-1]['height_rad']-target)<.7,'Phase requires review'
