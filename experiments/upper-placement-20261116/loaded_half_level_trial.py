import json,time,urllib.request,http.cookiejar,sys,numpy as np
from pathlib import Path
sys.path[:0]=["/home/arx/arx-competition-control","/home/arx/arx-competition-control/web/src"]
from control import arm_model
from agent.cabinet_preview_geometry import localize,opening
from agent.cabinet_loaded_sweep import check_outside_sweep
from fast_visual_level import table_front_x
root=Path("/home/arx/arx-place-hold-20261116");folder=Path(sys.argv[1])
read=lambda p:json.loads(Path(p).read_text())
axisdir=root/"reports/level-first-20261116-132201"
axis_report=read(axisdir/"axis-report.json")
reference=np.asarray(dict(np.load(axisdir/"2.npz"))["joints"])
half=reference+np.radians(np.asarray(axis_report["proposal"]["joint_delta_deg"]))*0.5
proposal=dict(path=dict(joint_waypoints_rad=[reference.tolist(),half.tolist()]))
frame=dict(np.load(folder/"d435.npz"));q0=np.array(proposal["path"]["joint_waypoints_rad"][0]);goal=np.array(proposal["path"]["joint_waypoints_rad"][-1])
base="http://127.0.0.1:9983";op=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()));op.open(base+"/",timeout=3).read()
def state():return json.load(op.open(base+"/api/state",timeout=2))
def send(data):
 assert data["type"] in ("arm.joints","arm.stop") and data["arm"]=="right"
 req=urllib.request.Request(base+"/api/command",data=json.dumps(data).encode(),headers={"Origin":base,"Content-Type":"application/json"})
 r=json.load(op.open(req,timeout=3))
 if r.get("ok") is not True:raise ValueError("No acknowledgement")
 return r
s=state();q=np.array(s["arms"]["right"]["joints"]);h=s["body"]["height"];g=s["arms"]["right"]["gripper"];epoch=s["command_epoch"];left=np.array(s["arms"]["left"]["joints"])
def guard(s):
 if not s["arms"]["right"]["fresh"] or not s["body"]["fresh"]:raise ValueError("Feedback stale")
 if s["motion_reserved"] or s["competition_reserved"] or s["chassis_active"] or s["command_epoch"]!=epoch:raise ValueError("Other control active")
 if abs(s["body"]["height"]-h)>.03 or abs(s["arms"]["right"]["gripper"]-g)>.05:raise ValueError("Body or clamp changed")
 if max(abs(np.array(s["arms"]["left"]["joints"])-left))>np.radians(.5):raise ValueError("Left arm moved")
guard(s)
if max(abs(q-q0))>np.radians(.15) or max(abs(goal-q))>np.radians(2.):raise ValueError("Proposal start changed or step too large")
if not 0<=time.time()-float(frame["image_time"])<=60:raise ValueError("Scene stale")
m=arm_model("right");tcp=np.array(read(root/"config/right-grip-tcp.json")["T_wrist_from_tool"])
b=read(root/"config/battery-envelope.json");b["contact_mm"]=read(root/"web/public/cad/battery-grip.json")["points_object_mm"]["tip"]
slot=opening(read(root/"config/cabinet-profiles.json"),"demo");window=localize(frame,profile=slot,now=float(frame["image_time"]))
sweep=check_outside_sweep(m,arm_model("left"),left,h,[q,goal],tcp,b,window,table_front_x(frame,details=True))
before=m.wrist_transform(q,h)@tcp;target=m.wrist_transform(goal,h)@tcp
if np.linalg.norm(target[:3,3]-before[:3,3])>.012:raise ValueError("Rotation TCP shift exceeds12mm")
guard(state());assert "--execute" in sys.argv, "Explicit --execute required";reply=send(dict(type="arm.joints",arm="right",joints=np.degrees(goal).tolist()));epoch+=1
print(json.dumps(dict(dispatch=reply,sweep=sweep["lower_mm"])),flush=True)
history=[];start=time.monotonic();result={}
try:
 while time.monotonic()-start<10:
  current=state();guard(current);row=current["arms"]["right"];actual=np.array(row["joints"]);pose=m.wrist_transform(actual,h)@tcp
  delta=(pose[:3,3]-before[:3,3])*1000;error=np.linalg.norm(pose[:3,3]-target[:3,3])*1000
  if np.linalg.norm(delta)>16 or max(abs(actual-q))>np.radians(2):raise ValueError("Unexpected motion")
  stamp=row["timestamp"]
  if not history or stamp>history[-1][0]:history.append((stamp,actual))
  history=[v for v in history if stamp-v[0]<.5]
  if max(abs(actual-goal))<np.radians(.65) and len(history)>3 and history[-1][0]-history[0][0]>.3 and np.max(np.ptp([v[1] for v in history],axis=0))<.003:
   result=dict(status="arrived",delta_base_mm=delta.tolist(),target_error_mm=float(error),state=current,gripper_commands=0);break
  time.sleep(.08)
 else:raise ValueError("Arrival timeout")
except Exception as e:
 result=dict(status="held",reason=str(e),stop=send(dict(type="arm.stop",arm="right")),gripper_commands=0)
(folder/"loaded-half-level-result.json").write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in result.items() if k!="state"},ensure_ascii=False),flush=True)
