import json,numpy as np
from pathlib import Path
from control import arm_model
from d405_partial import estimate_vertical_alignment
from vision_path_bridge import fine,native_plan
root=Path('/home/arx/arx-place-hold-20261116')
frame=dict(np.load(root/'reports/live-preview-20261116-121954-d198c162/d405.npz'))
conf=json.load(open('/home/arx/arx-competition-control/web/config/control.json'))
tcp=np.asarray(json.load(open(root/'config/right-grip-tcp.json'))['T_wrist_from_tool'])
old=json.load(open(root/'reports/live-preview-20261116-120237-afa2c723/nominal-candidate-2deg.json'))
plan=native_plan(old['alignment'])
assert set(plan)=={'interpolation','time_from_start_s','joint_waypoints_rad'}
print('native format:',len(plan['joint_waypoints_rad']),'points')
try:
 result=fine(frame,arm_model('right'),tcp,frame['joints'],float(frame['lift_rad']),float(frame['image_time'])-.01,float(frame['image_time']),conf['arms']['right']['joint_limits_deg'],old['alignment']['timing']['limits'],estimate_vertical_alignment)
 print('fine plan:',len(result['native_plan']['joint_waypoints_rad']),'points; correction',result['measurement']['step_mm'],'mm')
except ValueError as e:
 result={'status':'rejected','reason':str(e),'executable':False}
 print(result)
Path('saved-frame-integration-result.json').write_text(json.dumps(result,indent=2))
