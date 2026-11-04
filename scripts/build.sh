#!/bin/bash
set -eo pipefail
root=/home/arx/arx-competition-control
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPYCACHEPREFIX="$root/.pycache"
cd "$root"
python3 - <<'PY'
from pathlib import Path
import subprocess,concurrent.futures,json,os
r=Path.cwd();results=[]
pb=subprocess.check_output([str(r/'.venv/bin/python'),'-m','pybind11','--cmakedir'],text=True).strip()
def run(name,commands):
 with (r/'logs'/('build-'+name+'.log')).open('w') as log:
  for command in commands:
   log.write('COMMAND '+repr(command)+'\n');log.flush()
   p=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
   if p.returncode:return {'name':name,'returncode':p.returncode}
 return {'name':name,'returncode':0}
def pybuild():
 out=[]
 for name,src in [('x5','repos/X5/py/arx_x5_python/bimanual'),('lift','repos/LIFT/body/Python/ARX_LIFT_python/lift_controller')]:
  build=r/'build-python'/name
  commands=[['cmake','-S',str(r/src),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release','-Dpybind11_DIR='+pb,'-DPYTHON_EXECUTABLE='+str(r/'.venv/bin/python')],['cmake','--build',str(build),'--parallel','2']]
  out.append(run('python-'+name,commands))
 return out
ros=['colcon','--log-base',str(r/'workspace/log'),'build','--base-paths',str(r/'workspace/src'),'--build-base',str(r/'workspace/build'),'--install-base',str(r/'workspace/install'),'--executor','sequential','--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=OFF','-DCMAKE_BUILD_PARALLEL_LEVEL=2']
os.environ['CMAKE_BUILD_PARALLEL_LEVEL']='2'
with concurrent.futures.ThreadPoolExecutor(2) as pool:
 a=pool.submit(pybuild);b=pool.submit(run,'ros2',[ros]);results=a.result()+[b.result()]
(r/'reports/build.json').write_text(json.dumps(results,indent=2));print('BUILD_RESULTS',results,flush=True)
raise SystemExit(any(x['returncode'] for x in results))
PY
