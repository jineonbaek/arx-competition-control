"""Reusable image-edge leveling: observe -> bounded step -> settled reobserve.
No gripper commands. Original driver untouched. Default is observe-only.
"""
import argparse,json,os,subprocess,sys,time
from pathlib import Path
import numpy as np

def assess_change(before,after,outcome):
 allowed=outcome.get('status') in ('image_roll_step_arrived','image_edges_parallel') or (outcome.get('status')=='held' and outcome.get('reason')=='Measured arrival timeout')
 if not allowed:return 'hold_execution_failure'
 if not np.isfinite([before,after]).all():return 'hold_invalid_measurement'
 if abs(after)<=.75:return 'parallel'
 if abs(after)>abs(before)+.3:return 'hold_diverging'
 if abs(before)-abs(after)<.3:return 'hold_no_progress'
 return 'continue'

def stable_measurement(rows):
 if len(rows)<5 or len(set(v['sequence'] for v in rows))!=len(rows):return None
 if rows[-1]['captured_at']-rows[0]['captured_at']<.6:return None
 q=np.array([v['q'] for v in rows])
 if np.max(np.ptp(q,axis=0))>=np.radians(.12):return None
 angles=np.array([v['angle'] for v in rows]);good=abs(angles-np.median(angles))<=.35
 if good.sum()<5 or good.mean()<.7:return None
 valid=[v for v,k in zip(rows,good) if k]
 return dict(angle_deg=float(np.median(angles[good])),range_deg=float(np.ptp(angles[good])),frames=valid,rejected_frames=int((~good).sum()))

def empirical_command(error_deg,gain):
 if not np.isfinite([error_deg,gain]).all() or not .1<=abs(gain)<=3:raise ValueError('Invalid measured command response')
 return float(np.clip(-error_deg/gain,-2.,2.))

def main():
 import cv2,urllib.request
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');p.add_argument('--max-cycles',type=int,default=6);p.add_argument('--j6-command-gain',type=float);a=p.parse_args()
 if a.j6_command_gain is not None:empirical_command(1.,a.j6_command_gain)
 if not 1<=a.max_cycles<=6:raise ValueError('Cycle limit must be 1..6')
 here=Path(__file__).resolve().parent;work=Path('/home/arx/arx-place-hold-20261116');root=Path('/home/arx/arx-competition-control');overlay=work/'experiments/live-overlays-20261116'
 sys.path.insert(0,str(overlay));from battery_edges import detect_face
 from rim_edges import detect_rim,line_angle_error
 camera=root/'web/public/camera-live';base='http://127.0.0.1:9983'
 def state():return json.load(urllib.request.urlopen(base+'/api/state',timeout=2))
 initial=state();expected_epoch=initial['command_epoch'];held_joints=initial['arms']['right']['joints']
 def state_check(s):
  if not s['execution_enabled'] or not s['body']['fresh'] or not all(s['arms'][v]['fresh'] for v in ('left','right')):raise ValueError('Feedback unavailable')
  if s['command_epoch']!=expected_epoch:raise ValueError('Control epoch changed')
  if any(s[k] for k in ('motion_reserved','competition_reserved','chassis_active')):raise ValueError('Other control active')
  if abs(s['arms']['right']['gripper']-initial['arms']['right']['gripper'])>.05:raise ValueError('Gripper feedback changed')
  if abs(s['body']['height']-initial['body']['height'])>.03:raise ValueError('Lift moved')
  if max(abs(np.array(s['arms']['left']['joints'])-initial['arms']['left']['joints']))>np.radians(.5):raise ValueError('Left arm moved')
  if any(abs(s['body'][k]-initial['body'][k])>.02 for k in ('waist','head_yaw','head_pitch')):raise ValueError('Camera body changed')
 def observe(after):
  rows=[];seen=set();begin=time.monotonic()
  while time.monotonic()-begin<10:
   s=state();state_check(s);m=json.loads((camera/'d405.json').read_text())
   if m['sequence'] in seen or m['captured_at']<=after:time.sleep(.08);continue
   seen.add(m['sequence'])
   if m['state']!='online' or not 0<=time.time()-m['captured_at']<1.:raise ValueError('Camera stale')
   name=m['raw_image_file']
   if Path(name).name!=name:raise ValueError('Invalid frame file')
   im=cv2.imread(str(camera/name))
   if im is None:continue
   body=detect_face(im[:,:,::-1]);rim=detect_rim(im[:,:,::-1])
   if body is None or rim is None:continue
   angle=line_angle_error(body['quad_px'][:2],rim['edges']['top']['segment_px'])
   rows.append(dict(angle=angle,sequence=m['sequence'],captured_at=m['captured_at'],q=s['arms']['right']['joints'],height=s['body']['height']))
   rows=rows[-9:]
   settled=stable_measurement(rows)
   if settled is not None:return settled|dict(state=s)
   time.sleep(.05)
  (run/('observation-failed-'+str(time.time_ns())+'.json')).write_text(json.dumps(rows))
  raise ValueError('No settled repeatable five-frame edge measurement')
 run=work/'reports'/('edge-level-chain-'+time.strftime('%Y%m%d-%H%M%S'));run.mkdir();report=dict(status='observing',gripper_commands=0,cycles=[],execute=a.execute,j6_command_gain=a.j6_command_gain)
 def save(): (run/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
 env=os.environ.copy();env['PYTHONPATH']=os.pathsep.join(map(str,[root,root/'web/src',work/'experiments/placement-handoff-20261116',here]));python=str(root/'.venv/bin/python')
 try:
  before=observe(time.time());report['initial_observation']=before;save();print(json.dumps(dict(stage='observed',angle_deg=before['angle_deg'],range_deg=before['range_deg'])),flush=True)
  if not a.execute:report['status']='observed_only';return
  for i in range(a.max_cycles):
   if abs(before['angle_deg'])<=.75:report['status']='parallel';report['final_observation']=before;break
   state_check(state())
   command=empirical_command(before['angle_deg'],a.j6_command_gain) if a.j6_command_gain is not None else None
   cmd=[python,'-u',str(here/'loaded_coarse.py'),'--execute']
   cmd+=['--j6-step-deg',str(command),'--hold-joints',json.dumps(held_joints)] if command is not None else ['--align-top']
   child=subprocess.Popen(cmd,cwd=here,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True);lines=[];outcome=None
   for line in child.stdout:
    lines.append(line);print(line,end='',flush=True)
    try:value=json.loads(line)
    except ValueError:continue
    if 'status' in value:outcome=value
   code=child.wait();(run/('step-%02d.log'%i)).write_text(''.join(lines))
   if code or outcome is None:report['status']='hold_execution_failure';break
   if outcome.get('report'):
    execution=json.loads(Path(outcome['report']).read_text())
    if 'final_state' in execution:expected_epoch=execution['final_state']['command_epoch']
   after=observe(time.time());decision=assess_change(before['angle_deg'],after['angle_deg'],outcome)
   report['cycles'].append(dict(before=before,outcome=outcome,after=after,decision=decision,j6_command_deg=command));report['final_observation']=after;save()
   print(json.dumps(dict(step=i+1,before_deg=before['angle_deg'],after_deg=after['angle_deg'],decision=decision)),flush=True)
   if decision!='continue':report['status']=decision;break
   before=after
  else:report['status']='cycle_limit'
 except Exception as exc:report['status']='hold_observation_failure';report['reason']=str(exc)
 finally:
  save();print(json.dumps(dict(status=report['status'],report=str(run/'result.json'),angle_deg=report.get('final_observation',report.get('initial_observation',{})).get('angle_deg'),gripper_commands=0)),flush=True)
if __name__=='__main__':main()
