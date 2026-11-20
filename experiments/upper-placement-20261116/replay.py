"""Recorded-frame replay only. Does not open cameras, ROS, CAN, or control APIs."""
import argparse,json
from pathlib import Path
import numpy as np
from control import arm_model
from agent.cabinet_preview_geometry import opening
from d405_partial import estimate_vertical_alignment
from vision_path_bridge import coarse,fine
from upper_flow import OfflineFlow
from agent.visual_battery_level import motion_limits

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--workspace',default='/home/arx/arx-place-hold-20261116')
p.add_argument('--capture-dir',required=True)
p.add_argument('--stand-off-mm',type=float,default=30.)
p.add_argument('--control-config',default='/home/arx/arx-competition-control/web/config/control.json')
args=p.parse_args()
ROOT=Path(args.workspace)
read=lambda p:json.loads(Path(p).read_text())
folder=Path(args.capture_dir)
f435=dict(np.load(folder/'d435.npz'));f405=dict(np.load(folder/'d405.npz'))
config=read(args.control_config)
model=arm_model('right');tcp=np.asarray(read(ROOT/'config/right-grip-tcp.json')['T_wrist_from_tool'])
slot=opening(read(ROOT/'config/cabinet-profiles.json'),'demo');battery=read(ROOT/'config/battery-envelope.json')
contact=read(ROOT/'web/public/cad/battery-grip.json')['points_object_mm']['tip']
limits=config['arms']['right']['joint_limits_deg']
timing=motion_limits(read(ROOT/'config/battery-execution.json'),config)
report=dict(mode='historical_replay',robot_motion=False,executable=False,gripper_action='retain',capture_directory=str(folder))
driver=dict(driver_instance='SIMULATED-original-driver',command_epoch=0,status='idle',enabled=True,ready=True,manual_joint_active=False)
flow=OfflineFlow();flow.bind(driver,f435['joints'],float(f435['lift_rad']),float(f435['image_time']))
try:
    proposal=coarse(f435,model,tcp,slot,battery,contact,limits,timing,now=float(f435['image_time']),preclearance_mm=args.stand_off_mm)
    flow.stage_coarse(proposal);report['coarse']=proposal
    p=proposal['native_plan'];arrival=float(f435['image_time'])+p['time_from_start_s'][-1]
    flow.arrival(driver|dict(command_epoch=1,status='reference_complete'),p['joint_waypoints_rad'][-1],float(f435['lift_rad']),arrival)
    report['arrival']='SIMULATED endpoint only, not real hardware evidence'
    try:
        flow.check_fine_frame(f405,arrival)
        report['handoff']='unexpected acceptance'
        raise AssertionError('Pre-alignment frame must not pass handoff')
    except ValueError as exc:report['handoff']=dict(status='requires_new_post_arrival_d405',reason=str(exc))
except ValueError as exc:report['coarse']=dict(status='unavailable',reason=str(exc))
try:
    # Separate historical fine-planner check, NOT evidence of post-coarse arrival.
    report['fine_component']=fine(f405,model,tcp,f405['joints'],float(f405['lift_rad']),float(f405['image_time'])-.01,float(f405['image_time']),limits,timing,estimate_vertical_alignment)
    report['fine_component']['evidence']='Component replay only; arrival timestamp simulated'
except ValueError as exc:report['fine_component']=dict(status='unavailable',reason=str(exc))
report['flow']=flow.report()
Path('replay-report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(dict(coarse=report['coarse'].get('stage',report['coarse'].get('reason')),handoff=report.get('handoff'),fine=report['fine_component'].get('stage',report['fine_component'].get('reason')),flow=report['flow']),ensure_ascii=False))
