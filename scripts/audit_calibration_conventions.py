"""Offline convention audit. No serial/CAN/SDK hardware objects are opened."""
from pathlib import Path
import hashlib,json,math,struct,subprocess,xml.etree.ElementTree as ET,re
ROOT=Path('/home/arx/arx-competition-control')
AXES=['z+','y+','y-','y-','z-','x+']
def mmul(a,b):return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def rotation(rpy):
 r,p,y=rpy;cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
 return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
def model(path):
 accumulated=[[1,0,0],[0,1,0],[0,0,1]];joints=[]
 for j in ET.parse(path).getroot().findall('joint'):
  if j.attrib.get('name') not in ['joint'+str(i) for i in range(1,7)]:continue
  origin=j.find('origin');rpy=list(map(float,origin.attrib.get('rpy','0 0 0').split()));accumulated=mmul(accumulated,rotation(rpy))
  axis=list(map(float,j.find('axis').attrib['xyz'].split()));base=[sum(accumulated[i][k]*axis[k] for k in range(3)) for i in range(3)]
  joints.append({'joint':j.attrib['name'],'axis_in_joint_frame':axis,'axis_in_arm_base_at_q_zero':base,'origin_rpy':rpy,'origin_xyz':list(map(float,origin.attrib['xyz'].split()))})
 return joints

def run():
 sdk=ROOT/'repos/X5/py/arx_x5_python/bimanual/script/x5_2025.urdf';ros=ROOT/'repos/ARX_X5/ROS2/X5_ws/src/arx_x5_ros2/arx_x5_controller/x5_2025.urdf';cad=ROOT/'repos/ARX_Model/X5/X5A/urdf/X5A.urdf'
 a,b=model(sdk),model(ros)
 assert [j['axis_in_joint_frame'] for j in a]==[j['axis_in_joint_frame'] for j in b]
 assert a[2]['axis_in_joint_frame']==[0,1,0] and a[2]['axis_in_arm_base_at_q_zero'][1]<-0.9999
 for joint,axis in zip(a,AXES):joint['base_axis_rounded_label']=axis
 live=json.loads((ROOT/'reports/readonly-status-query.json').read_text());parameters=json.loads((ROOT/'reports/readonly-calibration-parameters.json').read_text())
 grippers={}
 for port,side in [('/dev/arxcan1','left'),('/dev/arxcan3','right')]:
  motor=next(x for x in live if x['port']==port)['motors']['8'];sample=motor['status_samples'][-1];count=sample['position_raw_u16'];device_max=motor['limits_read_from_device']['p_max'];sdk_angle=count*25/65535-12.5
  grippers[side]={'feedback_u16':count,'device_p_max_rad':device_max,'device_scale_angle_rad':sample['position_rad'],'sdk_fixed_scale_single_window_rad':sdk_angle,'scale_ratio_device_to_sdk':device_max/12.5,'constructor_initial_home_reference_rad':0.0,'effective_runtime_home_reference_rad':None,'getter_formula':'q_sdk_gripper = decoded_position - configured_home_position[6]','jaw_width_mm':None,'physical_opening_direction':None,'physical_width_calibration_verified':False}
 dm=[];encos=[]
 for bus in parameters:
  for p in bus['parameters']:
   item={'port':bus['port'],**p}
   (dm if p['protocol']=='DM' else encos).append(item)
 trace=(ROOT/'reports/readonly-calibration-parameters-syscalls.log').read_text()
 from readonly_calibration_parameters import allowed_commands
 writes=[]
 for line in trace.splitlines():
  if 'write(' not in line or '/dev/ttyACM' not in line:continue
  m=re.search(r'write\(\d+</dev/ttyACM([0-2])<[^>]+>>, "([^"\\]*(?:\\.[^"\\]*)*)", (\d+)',line)
  assert m,line
  dev,escaped,length=m.groups();port={'0':'/dev/arxcan1','1':'/dev/arxcan5','2':'/dev/arxcan3'}[dev];command=json.loads('"'+escaped+'"').encode()
  assert command in allowed_commands(port) and len(command)==int(length)
  writes.append({'port':port,'command':command.decode().strip()})
 assert len(writes)==54,len(writes)
 report={'scope':'static analysis and read-only existing parameter queries; no calibration actions','joint_mapping':{'motor_ids':[1,2,4,5,6,7],'default_sdk_software_signs':[1,1,1,1,1,1],'first_six_sdk_getter_additional_offsets_rad':[0,0,0,0,0,0],'axes':a,'zero_definition':'q=0 means firmware-referenced output angle zero; home motion target is separate','physical_zero_alignment_verified':False,'physical_direction_verified_by_motion':False,'left_right_base_extrinsics_verified':False,'turn_history':'SDK decoders may unwrap angles; a static sample does not establish past turn count'},'gripper':grippers,'gripper_cad':{'model_variant':'X5A; not proven identical to installed 2025 gripper','two_prismatic_joint_strokes_m':[0.044,0.044],'maximum_total_gap_change_m':0.088,'motor_to_slider_transmission_provided':False,'absolute_jaw_gap_from_origins_is_valid':False},'lift':{'sdk_feedback_angle_formula':'H_angle_rad = -q_lift_motor_rad','ros2_setHeight_input_unit':'motor angle rad','ros2_feedback_unit':'motor angle rad','python_set_height_scaling_rad_per_nominal_m':41.54,'python_feedback_is_meters':False,'nominal_relative_formula':'delta_h_m = -(q_motor - q_motor_reference)/41.54','nominal_20_rad_stroke_m':20/41.54,'absolute_height_m':None,'physical_zero_height_verified':False,'physical_scale_verified':False},'existing_dm_parameters':dm,'encos_zero_queries':encos,'audit':{'serial_writes_checked':len(writes),'only_allowlisted_parameter_reads_and_adapter_config':True,'zeroing_performed':False,'movement_commands_sent':False,'hardware_sdk_instantiated':False}}
 files=[sdk,ros,cad,ROOT/'repos/X5/py/arx_x5_python/bimanual/lib/arx_x5_src/libarx_x5_src.so',ROOT/'repos/ARX_X5/ROS2/X5_ws/src/arx_x5_ros2/arx_x5_controller/lib/arx_x5_src/libarx_x5_src.so',ROOT/'repos/LIFT/body/Python/ARX_LIFT_python/lift_controller/lib/arx_lift_src/include/arx_lift_src/lift_head_control_loop.h']
 report['evidence_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
 assert all(g['jaw_width_mm'] is None for g in grippers.values()) and report['lift']['absolute_height_m'] is None
 (ROOT/'reports/calibration-conventions.json').write_text(json.dumps(report,indent=2))
 profile={'status':'NOT_PHYSICALLY_CALIBRATED','hardware_execution_enabled':False,'joint_physical_zero_offsets_rad':{'left':[None]*6,'right':[None]*6},'joint_physical_direction_validated':False,'gripper_width_calibration':{'left':None,'right':None},'lift_height_reference_m':None,'lift_motor_angle_reference_rad':None,'nominal_lift_rad_per_m':41.54,'apply_nominal_scale_without_reference':False}
 (ROOT/'reports/calibration-profile-unverified.json').write_text(json.dumps(profile,indent=2))
 print('CONVENTIONS_AUDITED: six joint axes, two SDK versions, two grippers, lift units')
 print('BASE_AXES_AT_ZERO',AXES);print('GRIPPER',json.dumps(grippers));print('NOMINAL_LIFT_20RAD_M',20/41.54);print('SERIAL_READ_AUDIT',len(writes));print('PHYSICAL_CALIBRATION_STAYS_UNVERIFIED')
if __name__=='__main__':run()
