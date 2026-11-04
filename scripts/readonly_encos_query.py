"""ENCOS manual V1.19 sections 9.3/10.5, queries only (E1 code).
No motor initialization, control/configuration frames or SDK objects.
"""
from pathlib import Path
import json,math,struct,time,subprocess
import serial
from readonly_motor_query import ROOT,read_until,ADAPTER_COMMANDS
from readonly_status_query import parse_frames
TARGETS={'/dev/arxcan1':[1,2,4],'/dev/arxcan3':[1,2,4]}
CODES={1:'position',2:'velocity',3:'current',30:'version'}
def query_request(path,motor_id,code):
 if motor_id not in TARGETS.get(path,[]) or code not in CODES:raise ValueError('Target/code outside documented query allowlist')
 return f't{motor_id:03X}2E1{code:02X}\r'.encode()
def validate_write(path,command):
 if path not in TARGETS:raise ValueError('Unknown adapter')
 allowed=ADAPTER_COMMANDS|{query_request(path,motor,code) for motor in TARGETS[path] for code in CODES}
 if command not in allowed:raise ValueError('Only fixed adapter commands and E1 queries allowed')
def decode_reply(frame,motor_id,code):
 data=bytes.fromhex(frame['data_hex'])
 if frame['can_id']!=motor_id or len(data)<2 or data[0]>>5!=5 or data[1]!=code:return None
 result={'raw_frame':frame,'error_code':data[0]&31}
 if code in [1,2,3] and len(data)==6:
  value=struct.unpack('>f',data[2:6])[0]
  if not math.isfinite(value):return None
  if code==1:result.update(position_deg=value,position_rad=math.radians(value))
  elif code==2:result.update(velocity_rpm=value,velocity_rad_s=value*2*math.pi/60)
  else:result.update(current_A=value)
 elif code==30 and len(data)==8:result['version_bytes_hex']=data[2:].hex()
 else:return None
 return result

def run():
 reports=[]
 for path,motors in TARGETS.items():
  if subprocess.run(['fuser',path],capture_output=True).returncode==0:raise RuntimeError('Port occupied')
  report={'port':path,'events':[],'motors':{},'mode':'ENCOS-E1-parameter-query-only'}
  with serial.Serial(path,115200,timeout=0.02,write_timeout=0.5,exclusive=True) as port:
   def send(command,wait=0.15):
    validate_write(path,command);port.write(command);port.flush();raw=read_until(port,wait)
    event={'time_ns':time.time_ns(),'tx_ascii':command.decode().strip(),'rx_hex':raw.hex(),'frames':parse_frames(raw)};report['events'].append(event)
    return event
   try:
    for command in [b'C\r',b'V\r',b'S8\r',b'O\r']:send(command)
    for motor in motors:
     samples=[]
     for code in [30,1,2,3,1,2,1,2]:
      event=send(query_request(path,motor,code),0.2)
      for frame in event['frames']:
       decoded=decode_reply(frame,motor,code)
       if decoded:samples.append({'time_ns':event['time_ns'],'query_code':code,**decoded})
     report['motors'][str(motor)]={'samples':samples,'position_reply_count':sum('position_rad' in s for s in samples)}
   finally:send(b'C\r')
  reports.append(report)
 (ROOT/'reports/readonly-encos-query.json').write_text(json.dumps(reports,indent=2))
 for report in reports:
  print('ENCOS_RESULT',report['port'],json.dumps(report['motors']),flush=True)
if __name__=='__main__':run()
