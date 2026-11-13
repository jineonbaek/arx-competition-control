#!/usr/bin/env python3
import os,sys,time,signal,subprocess,json
from pathlib import Path
root=Path(__file__).resolve().parents[1];report=Path(sys.argv[1]);report.mkdir(parents=True,exist_ok=True)
profile=sys.argv[2] if len(sys.argv)>2 else 'baseline15'
profiles={'baseline15':('640,480,15','640,480,15',True,20),'balanced30':('640,480,30','848,480,30',True,30),'depth90':('640,480,30','848,480,90',False,30),'full30':('1920,1080,30','1280,720,30',True,30)}
profiles['fast90']=('640,480,60','848,480,90',True,30)
color,depth,color_on,duration=profiles[profile]
duration=float(os.environ.get('SENSOR_TEST_DURATION',duration))
children=[];logs=[]
def spawn(command,name):
 f=(report/(profile+'-'+name+'.log')).open('w');logs.append(f);p=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=os.environ.copy());children.append(p);return p
try:
 camera=['ros2','launch','realsense2_camera','rs_launch.py','camera_namespace:=sensors','camera_name:=d435','serial_no:=_349622074226','enable_color:='+str(color_on).lower(),'enable_depth:=true','enable_infra1:=false','enable_infra2:=false','rgb_camera.color_profile:='+color,'depth_module.depth_profile:='+depth]
 if os.environ.get('SENSOR_QOS'):
  camera+=['color_qos:='+os.environ['SENSOR_QOS'],'depth_qos:='+os.environ['SENSOR_QOS']]
 spawn(camera,'camera')
 spawn(['ros2','run','livox_ros_driver2','livox_ros_driver2_node','--ros-args','-r','__node:=dual_mid360s_benchmark','-p','xfer_format:=1','-p','multi_topic:=1','-p','data_src:=0','-p','publish_freq:=10.0','-p','output_data_type:=0','-p','frame_id:=livox_unmounted','-p','user_config_path:='+str(root/'config/mid360s_dual_capture.json')],'lidars')
 time.sleep(6)
 probe=[str(root/'build-tools/sensor_rate_probe/sensor_rate_probe'),'--ros-args','-p','duration:='+str(float(duration)),'-p','output:='+str(report/(profile+'-rates.csv'))]
 if os.environ.get('PROBE_RELIABILITY'):probe+=['-p','reliability:='+os.environ['PROBE_RELIABILITY']]
 result=subprocess.run(probe,stdout=(report/(profile+'-probe.log')).open('w'),stderr=subprocess.STDOUT,timeout=duration+20)
 (report/(profile+'-test.json')).write_text(json.dumps({'profile':profile,'color_profile':color,'depth_profile':depth,'color_enabled':color_on,'duration_s':duration,'probe_returncode':result.returncode,'middleware':os.environ.get('RMW_IMPLEMENTATION','default'),'transports':os.environ.get('FASTDDS_BUILTIN_TRANSPORTS','default'),'sensor_qos':os.environ.get('SENSOR_QOS','default'),'probe_reliability':os.environ.get('PROBE_RELIABILITY','best_effort'),'xml_profile':os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE','none')},indent=2))
 print((report/(profile+'-rates.csv')).read_text() if (report/(profile+'-rates.csv')).exists() else 'No rate output',flush=True)
finally:
 for p in children:
  if p.poll() is None:
   try:os.killpg(p.pid,signal.SIGINT)
   except ProcessLookupError:pass
 for p in children:
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=4)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for f in logs:f.close()
