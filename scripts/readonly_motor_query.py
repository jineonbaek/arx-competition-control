"""Bounded parameter reads using the documented Damiao 0x33 request.
No SDK hardware objects, enable/disable, setpoint, parameter write or save.
Known target: platform MotorType3 lift motor ID 1 (ARX library constructor).
"""
from pathlib import Path
import json,re,subprocess,time
import serial
ROOT=Path('/home/arx/arx-competition-control')
TARGETS={'/dev/arxcan5':1}
REGISTERS={13:'hardware_version',14:'software_version'}
ADAPTER_COMMANDS={b'C\r',b'S8\r',b'O\r',b'V\r'}
def read_request(path,motor_id,register):
 if TARGETS.get(path)!=motor_id or register not in REGISTERS:raise ValueError('Target/register outside finite read-only allowlist')
 payload=bytes([motor_id,0,0x33,register,0,0,0,0])
 return ('t7FF8'+payload.hex().upper()+'\r').encode('ascii')
def validate_serial_write(path,data):
 if path not in TARGETS:raise ValueError('Unknown adapter')
 allowed=ADAPTER_COMMANDS|{read_request(path,TARGETS[path],rid) for rid in REGISTERS}
 if data not in allowed:raise ValueError('Only fixed adapter operations and 0x33 version reads are permitted')
def read_until(port,seconds):
 end=time.monotonic()+seconds;out=b''
 while time.monotonic()<end:out+=port.read(65536)
 return out

def run():
 reports=[]
 for path,motor_id in TARGETS.items():
  if subprocess.run(['fuser',path],capture_output=True).returncode==0:raise RuntimeError('Adapter in use')
  report={'port':path,'motor_id':motor_id,'events':[],'motor_enable_sent':False,'control_frames_sent':False}
  with serial.Serial(path,115200,timeout=0.05,write_timeout=0.5,exclusive=True) as port:
   def send(data,wait=0.3):
    validate_serial_write(path,data);port.write(data);port.flush();reply=read_until(port,wait)
    event={'time_ns':time.time_ns(),'tx_ascii':data.decode().strip(),'rx_hex':reply.hex()};report['events'].append(event)
    print('READONLY_QUERY',path,event,flush=True)
   try:
    for cmd in [b'C\r',b'V\r',b'S8\r',b'O\r']:send(cmd)
    for rid in REGISTERS:send(read_request(path,motor_id,rid),1.0)
   finally:send(b'C\r')
  reports.append(report)
 (ROOT/'reports/readonly-motor-version-probe.json').write_text(json.dumps(reports,indent=2))
if __name__=='__main__':run()
