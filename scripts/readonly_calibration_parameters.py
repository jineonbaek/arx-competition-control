"""Read existing calibration parameters only; never set zero or write registers."""
import json,struct,time,math,subprocess
import serial
from readonly_motor_query import ROOT,read_until,ADAPTER_COMMANDS
from readonly_status_query import parse_frames
DM_IDS={'/dev/arxcan1':[5,6,7,8],'/dev/arxcan3':[5,6,7,8],'/dev/arxcan5':[1]}
ENCOS_IDS={'/dev/arxcan1':[1,2,4],'/dev/arxcan3':[1,2,4]}
DM_REGS={20:'Gr',54:'m_off',55:'dir'}
def dm_request(m,rid):return ('t7FF8'+bytes([m,0,0x33,rid,0,0,0,0]).hex().upper()+'\r').encode()
def encos_request(m,code):return f't{m:03X}2E1{code:02X}\r'.encode()
def allowed_commands(path):
 if path not in DM_IDS:raise ValueError('Unknown adapter')
 return ADAPTER_COMMANDS|{dm_request(m,rid) for m in DM_IDS[path] for rid in DM_REGS}|{encos_request(m,c) for m in ENCOS_IDS.get(path,[]) for c in [38,39]}
def run():
 result=[]
 for path in DM_IDS:
  if subprocess.run(['fuser',path],capture_output=True).returncode==0:raise RuntimeError('Port occupied')
  report={'port':path,'events':[],'parameters':[]}
  with serial.Serial(path,115200,timeout=0.02,write_timeout=0.5,exclusive=True) as port:
   def send(command):
    if command not in allowed_commands(path):raise ValueError('Forbidden command')
    port.write(command);port.flush();raw=read_until(port,0.22)
    event={'time_ns':time.time_ns(),'tx_ascii':command.decode().strip(),'rx_hex':raw.hex(),'frames':parse_frames(raw)}
    report['events'].append(event);return event['frames']
   try:
    for command in [b'C\r',b'V\r',b'S8\r',b'O\r']:send(command)
    for motor in DM_IDS[path]:
     for rid in DM_REGS:
      matches=[]
      for frame in send(dm_request(motor,rid)):
       d=bytes.fromhex(frame['data_hex'])
       if len(d)==8 and d[:4]==bytes([motor,0,0x33,rid]):
        f=struct.unpack('<f',d[4:])[0];matches.append({'raw_hex':d[4:].hex(),'as_uint32':struct.unpack('<I',d[4:])[0],'as_float32':f if math.isfinite(f) else None})
      report['parameters'].append({'protocol':'DM','motor_id':motor,'register':rid,'name':DM_REGS[rid],'replies':matches})
    for motor in ENCOS_IDS.get(path,[]):
     for code in [38,39]:
      matches=[]
      for frame in send(encos_request(motor,code)):
       d=bytes.fromhex(frame['data_hex'])
       if frame['can_id']!=motor or len(d)<2 or d[0]>>5!=5 or d[1]!=code:continue
       v={'raw_frame':frame,'error_code':d[0]&31}
       if code==38 and len(d)==4:v['zero_offset_deg']=struct.unpack('>h',d[2:4])[0]/100
       if code==39 and len(d)==7:v.update(encoder_calibration_flag=d[2],encoder_raw_deg=struct.unpack('>h',d[3:5])[0]/100,encoder_calibrated_deg=struct.unpack('>h',d[5:7])[0]/100)
       matches.append(v)
      report['parameters'].append({'protocol':'ENCOS','motor_id':motor,'query_code':code,'replies':matches})
   finally:send(b'C\r')
  result.append(report)
 (ROOT/'reports/readonly-calibration-parameters.json').write_text(json.dumps(result,indent=2))
 for report in result:print('CALIBRATION_PARAMETERS',report['port'],json.dumps(report['parameters']),flush=True)
if __name__=='__main__':run()
