"""Aggregate recorded queries without opening any hardware."""
from pathlib import Path
import json,math,os,re,subprocess
from readonly_status_query import validate_write as validate_dm
from readonly_encos_query import validate_write as validate_encos
ROOT=Path('/home/arx/arx-competition-control')
def audit(name,validator):
 text=(ROOT/'reports'/name).read_text();writes=[]
 for line in text.splitlines():
  match=re.search(r'write\(\d+</dev/ttyACM([0-2])<[^>]+>>, "([^"\\]*(?:\\.[^"\\]*)*)", (\d+)',line)
  if not match:continue
  device,escaped,length=match.groups();port={'0':'/dev/arxcan1','1':'/dev/arxcan5','2':'/dev/arxcan3'}[device]
  command=json.loads('"'+escaped+'"').encode('ascii');validator(port,command)
  assert int(length)==len(command)
  writes.append({'port':port,'command':command.decode().strip()})
 assert writes,'No adapter writes found in audit'
 return writes

def summarize():
 dm=json.loads((ROOT/'reports/readonly-status-query.json').read_text());encos=json.loads((ROOT/'reports/readonly-encos-query.json').read_text())
 result={'mode':'live-query-only','coordinate_convention':'raw motor feedback before robot joint sign/offset calibration','arms':{},'lift':{},'calibrated_lift_height_m':None,'simultaneous_snapshot':False}
 dm_by_port={x['port']:x for x in dm};encos_by_port={x['port']:x for x in encos}
 for side,port in [('left','/dev/arxcan1'),('right','/dev/arxcan3')]:
  joints=[]
  for joint,motor in enumerate([1,2,4,5,6,7],1):
   if motor in [1,2,4]:
    samples=encos_by_port[port]['motors'][str(motor)]['samples'];pos=[x for x in samples if 'position_rad' in x];vel=[x['velocity_rad_s'] for x in samples if 'velocity_rad_s' in x]
    errors=sorted(set(x['error_code'] for x in samples));enabled=None
   else:
    pos=dm_by_port[port]['motors'][str(motor)]['status_samples'];vel=[x['velocity_rad_s'] for x in pos];errors=None;enabled=sorted(set(x['status_nibble'] for x in pos))
   values=[x['position_rad'] for x in pos]
   assert len(values)==3 and all(math.isfinite(x) for x in values)
   joints.append({'joint':joint,'motor_id':motor,'position_rad':values[-1],'sample_count':len(values),'position_span_rad':max(values)-min(values),'max_abs_reported_velocity_rad_s':max(abs(v) for v in vel),'encos_error_codes':errors,'damiao_status_nibbles':enabled})
  gripper=dm_by_port[port]['motors']['8']['status_samples'];values=[x['position_rad'] for x in gripper]
  assert len(values)==3
  result['arms'][side]={'joints':joints,'gripper':{'motor_id':8,'position_rad':values[-1],'sample_count':3,'position_span_rad':max(values)-min(values),'jaw_width_mm':None,'damiao_status_nibbles':sorted(set(x['status_nibble'] for x in gripper))}}
 lift=dm_by_port['/dev/arxcan5']['motors']['1']['status_samples'];values=[x['position_rad'] for x in lift]
 assert len(values)==3
 result['lift']={'motor_id':1,'motor_position_rad':values[-1],'sdk_height_raw_angle_rad':-values[-1],'sample_count':3,'position_span_rad':max(values)-min(values),'damiao_status_nibbles':sorted(set(x['status_nibble'] for x in lift)),'mos_temperature_C':lift[-1]['temperature_mos_C'],'motor_temperature_C':lift[-1]['temperature_rotor_C']}
 dm_writes=audit('readonly-status-query-syscalls.log',lambda p,c:validate_dm(p,c,True))
 encos_writes=audit('readonly-encos-query-syscalls.log',validate_encos)
 assert len(dm_writes)==87 and len(encos_writes)==58, 'Incomplete serial write audit'
 result['audit']={'dm_write_count':len(dm_writes),'encos_write_count':len(encos_writes),'all_writes_match_readonly_allowlists':True,'motor_enable_or_disable_commands_sent':False,'setpoints_sent':False,'register_write_save_clear_zero_commands_sent':False}
 result['feedback_channels_verified']=15;result['position_samples_verified']=45
 result['all_feedback_verified']=True
 result['cleanup']={'serial_device_owners':subprocess.run(['fuser','/dev/arxcan1','/dev/arxcan3','/dev/arxcan5'],capture_output=True,text=True).stdout.strip(),'can_interfaces':subprocess.check_output(['ip','-brief','link','show','type','can'],text=True).strip(),'hardware_driver_processes':subprocess.run(['pgrep','-af','X5Controller|/lift_controller|slcand|candump'],capture_output=True,text=True).stdout.strip()}
 assert not any(result['cleanup'].values()),result['cleanup']
 (ROOT/'reports/live-readonly-status.json').write_text(json.dumps(result,indent=2))
 print('LIVE_READONLY_VERIFIED',result['feedback_channels_verified'],'channels',result['position_samples_verified'],'position samples')
 for side in ['left','right']:
  print(side,'raw_joint_rad',[round(x['position_rad'],6) for x in result['arms'][side]['joints']],'gripper_rad',round(result['arms'][side]['gripper']['position_rad'],6))
 print('LIFT',result['lift']);print('AUDIT',result['audit']);print('CLEANUP',result['cleanup'])
if __name__=='__main__':summarize()
