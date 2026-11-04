"""Finite documented DM register reads and status refresh; no control object."""
from pathlib import Path
import json,re,struct,math,time,subprocess
import serial
from readonly_motor_query import read_until,ROOT,ADAPTER_COMMANDS
TARGETS={'/dev/arxcan5':[1],'/dev/arxcan1':[5,6,7,8],'/dev/arxcan3':[5,6,7,8]}
REGISTERS={13:'hw_ver',14:'sw_ver',21:'p_max',22:'v_max',23:'t_max'}
def register_request(path,motor_id,rid):
 if motor_id not in TARGETS.get(path,[]) or rid not in REGISTERS:raise ValueError('Not in fixed target/register allowlist')
 return ('t7FF8'+bytes([motor_id,0,0x33,rid,0,0,0,0]).hex().upper()+'\r').encode()
def status_request(path,motor_id,identified):
 if not identified or motor_id not in TARGETS.get(path,[]):raise ValueError('Identity not verified')
 return ('t7FF8'+bytes([motor_id,0,0xCC,0,0,0,0,0]).hex().upper()+'\r').encode()
def validate_write(path,command,identified):
 if path not in TARGETS:raise ValueError('Unknown port')
 allowed=set(ADAPTER_COMMANDS)
 for motor in TARGETS[path]:
  allowed|={register_request(path,motor,rid) for rid in ([13,14,21,22,23] if identified else [13,14])}
  if identified:allowed.add(status_request(path,motor,True))
 if command not in allowed:raise ValueError('Only fixed register reads and identity-gated refresh requests allowed')
def parse_frames(raw):
 frames=[]
 for line in raw.decode('ascii',errors='replace').split('\r'):
  m=re.fullmatch(r't([0-9A-Fa-f]{3})([0-8])([0-9A-Fa-f]*)',line)
  if not m:continue
  identifier,dlc,data=m.groups();n=int(dlc)
  if len(data) not in [n*2,n*2+4]:continue
  frames.append({'can_id':int(identifier,16),'data_hex':data[:n*2].upper()})
 return frames

def run():
 reports=[]
 for path,motor_ids in TARGETS.items():
  if subprocess.run(['fuser',path],capture_output=True).returncode==0:raise RuntimeError('Port occupied')
  report={'port':path,'events':[],'motors':{},'mode':'parameter-read-and-refresh-only'}
  with serial.Serial(path,115200,timeout=0.05,write_timeout=0.5,exclusive=True) as port:
   identified=False
   def send(command,wait=0.3):
    validate_write(path,command,identified);port.write(command);port.flush();response=read_until(port,wait)
    event={'time_ns':time.time_ns(),'tx_ascii':command.decode().strip(),'rx_hex':response.hex(),'frames':parse_frames(response)}
    report['events'].append(event);return event['frames']
   try:
    for command in [b'C\r',b'V\r',b'S8\r',b'O\r']:send(command)
    for motor in motor_ids:
     identified=False;values={};status=[]
     for rid in [13,14]:
      for frame in send(register_request(path,motor,rid),0.4):
       data=bytes.fromhex(frame['data_hex'])
       if len(data)==8 and data[:4]==bytes([motor,0,0x33,rid]):values[rid]=data[4:]
     identified=13 in values and 14 in values
     if identified:
      for rid in [21,22,23]:
       for frame in send(register_request(path,motor,rid),0.4):
        data=bytes.fromhex(frame['data_hex'])
        if len(data)==8 and data[:4]==bytes([motor,0,0x33,rid]):values[rid]=data[4:]
      limits={REGISTERS[rid]:struct.unpack('<f',values[rid])[0] for rid in [21,22,23] if rid in values}
      for index in range(3):
       for frame in send(status_request(path,motor,True),0.4):
        data=bytes.fromhex(frame['data_hex'])
        if len(data)!=8 or (data[0]&15)!=motor:continue
        p=(data[1]<<8)|data[2];v=(data[3]<<4)|(data[4]>>4);t=((data[4]&15)<<8)|data[5]
        sample={'time_ns':time.time_ns(),'raw_frame':frame,'status_nibble':data[0]>>4,'position_raw_u16':p,'velocity_raw_u12':v,'torque_raw_u12':t,'temperature_mos_C':data[6],'temperature_rotor_C':data[7]}
        for name,value,maxkey,bits in [('position_rad',p,'p_max',16),('velocity_rad_s',v,'v_max',12),('torque_protocol_units',t,'t_max',12)]:
         maximum=limits.get(maxkey)
         sample[name]=(value*2*maximum/(2**bits-1)-maximum) if maximum is not None and math.isfinite(maximum) and maximum>0 else None
        status.append(sample)
     else:limits={}
     report['motors'][str(motor)]={'identity_verified':identified,'register_hex':{REGISTERS[k]:v.hex() for k,v in values.items()},'limits_read_from_device':limits,'status_samples':status}
   finally:send(b'C\r')
  reports.append(report)
 (ROOT/'reports/readonly-status-query.json').write_text(json.dumps(reports,indent=2))
 for report in reports:
  print('STATUS_RESULT',report['port'],json.dumps(report['motors']),flush=True)
if __name__=='__main__':run()
