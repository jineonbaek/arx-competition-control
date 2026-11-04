from pathlib import Path
import subprocess,os,json,hashlib
r=Path('/home/arx/arx-competition-control');out=r/'reports/lift-unit-chain';out.mkdir(exist_ok=True)
roots={'original':Path('/home/arx/LIFT/body/ROS2'),'isolated':r/'workspace'}
result=[]
for key,root in roots.items():
 exe=root/'install/arx_lift_controller/lib/arx_lift_controller/lift_controller'
 libs=[root/'install/arx_lift_controller/lib',root/'install/arm_control/lib',root/'install/arx5_arm_msg/lib',Path('/opt/ros/jazzy/lib')]
 env={'PATH':'/usr/bin:/bin','HOME':'/home/arx','LANG':'C.UTF-8','LD_LIBRARY_PATH':':'.join(map(str,libs))}
 cmd=['strace','-f','-yy','-e','trace=openat,socket,connect,ioctl,write,execve','-o',str(out/(key+'-loader-strace.log')),'/lib64/ld-linux-x86-64.so.2','--list',str(exe)]
 p=subprocess.run(cmd,env=env,capture_output=True,text=True,timeout=30)
 (out/(key+'-loader-list.txt')).write_text(p.stdout+p.stderr)
 print(key,'LOADER_RC',p.returncode)
 match=[s.strip() for s in p.stdout.splitlines() if 'libarx_lift_src.so =>' in s];print(match)
 if p.returncode!=0:print(p.stderr[:300])
 resolved=Path(match[0].split('=>')[1].strip().split(' ')[0]) if match else None
 rec={'case':key,'executable':str(exe),'environment':env,'returncode':p.returncode,'resolved_library':str(resolved) if resolved else None,'sha256':hashlib.sha256(resolved.read_bytes()).hexdigest() if resolved else None,'runtime_process_observation':False,'control_main_executed':False}
 result.append(rec)
 for stage,pth in [('installed',exe),('build',root/'build/arx_lift_controller/lift_controller')]:
  if not pth.exists():continue
  subprocess.run(['objcopy','--dump-section','.text='+str(out/(key+'-'+stage+'-text.bin')),str(pth),str(out/(key+'-'+stage+'-elf-copy'))],check=True)
  s=subprocess.check_output(['objdump','-d','-C',str(pth)],text=True)
  (out/(key+'-'+stage+'-static.asm.txt')).write_text(s)
  print(key,stage,'TEXT_SHA',hashlib.sha256((out/(key+'-'+stage+'-text.bin')).read_bytes()).hexdigest())
(out/'loader-resolution.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print('ONLY_LOADER_LIST_MODE_EXECUTED')
