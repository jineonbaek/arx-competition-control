"""Offline checks only. Never construct arm, encoder or platform hardware objects."""
from pathlib import Path
import importlib, json, os, subprocess, sys
import numpy as np
from rclpy.serialization import serialize_message, deserialize_message
from ament_index_python.packages import get_package_prefix
from arx5_arm_msg.msg import RobotStatus
from arm_control.msg import PosCmd
root=Path('/home/arx/arx-competition-control')
checks=[]
def check(name, condition, detail):
    checks.append({'name':name,'passed':bool(condition),'detail':detail})
    print(('PASS ' if condition else 'FAIL ')+name,flush=True)
modules={name:importlib.import_module(name) for name in ['bimanual','arx_x5_python','kinematic_solver','encoder_interface','arx_lift_python','lift_controller']}
check('independent_sdk_imports',all(str(Path(m.__file__).resolve()).startswith(str(root)+'/') for m in modules.values()),{k:m.__file__ for k,m in modules.items()})
arm=modules['arx_x5_python'].InterfacesPy
arm_methods=['set_joint_positions','set_ee_pose','set_arm_status','set_catch','mit_joint_control','get_joint_positions','get_joint_velocities','get_joint_currents','get_ee_pose']
check('arm_interfaces_present',all(hasattr(arm,x) for x in arm_methods),arm_methods)
lift=modules['arx_lift_python'].LiftHeadControlLoop
lift_methods=['set_height','get_height','set_chassis_cmd','set_head_yaw','set_head_pitch','loop']
check('platform_interfaces_present',all(hasattr(lift,x) for x in lift_methods),lift_methods)
from bimanual.script.single_arm import euler_to_quaternion, quaternion_to_euler
angles=np.array([0.1,-0.2,0.3]);recovered=np.array(quaternion_to_euler(euler_to_quaternion(*angles)))
check('pose_conversion_roundtrip',np.allclose(angles,recovered,atol=1e-10),recovered.tolist())
solver=modules['kinematic_solver'].KinematicSolver()
pose=solver.forward_kinematics(np.zeros(6,dtype=np.float64))
check('offline_forward_kinematics',pose.shape==(6,) and np.isfinite(pose).all(),pose.tolist())
msg=RobotStatus();msg.joint_pos=[0.1*i for i in range(7)];msg.header.stamp.sec=123
back=deserialize_message(serialize_message(msg),RobotStatus)
check('arm_message_serialization',list(back.joint_pos)==list(msg.joint_pos) and back.header.stamp.sec==123,RobotStatus.get_fields_and_field_types())
msg2=PosCmd();msg2.height=0.123;msg2.mode1=3
back2=deserialize_message(serialize_message(msg2),PosCmd)
check('platform_message_serialization',back2.height==0.123 and back2.mode1==3,PosCmd.get_fields_and_field_types())
prefixes={name:get_package_prefix(name) for name in ['arm_control','arx5_arm_msg','arx_x5_controller','arx_lift_controller']}
check('independent_ros_packages',all(str(Path(p).resolve()).startswith(str(root)+'/') for p in prefixes.values()),prefixes)
binaries=[Path(modules[n].__file__) for n in ['arx_x5_python','kinematic_solver','encoder_interface','arx_lift_python']]+[root/'workspace/install/arx_x5_controller/lib/arx_x5_controller/X5Controller',root/'workspace/install/arx_lift_controller/lib/arx_lift_controller/lift_controller']
missing={}
for binary in binaries:
    result=subprocess.run(['ldd',str(binary)],capture_output=True,text=True)
    errors=[line.strip() for line in result.stdout.splitlines() if 'not found' in line]
    if result.returncode or errors:missing[str(binary)]=errors+[result.stderr]
check('native_dependencies_resolved',not missing,missing)
contaminated={k:v for k,v in os.environ.items() if k in ['PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH'] and any(p in v for p in ['/home/arx/LIFT/','/home/arx/ROS2_LIFT_Play/','/home/arx/docs_lift2/'])}
check('no_old_project_environment',not contaminated,contaminated)
clean={name:subprocess.check_output(['git','status','--porcelain','--untracked-files=normal'],cwd=root/'repos'/name,text=True) for name in ['X5','LIFT','ARX_X5']}
check('official_sources_clean',not any(clean.values()),clean)
report={'mode':'offline_only','hardware_objects_created':False,'checks':checks,'passed':all(c['passed'] for c in checks),'python':sys.version,'sources':json.loads((root/'reports/sources.json').read_text())}
(root/'reports/offline-check.json').write_text(json.dumps(report,indent=2))
print('RESULT',sum(c['passed'] for c in checks),'/',len(checks),flush=True)
raise SystemExit(not report['passed'])
