"""Install already-built SDK modules into this independent directory only."""
from pathlib import Path
import shutil,subprocess
r=Path('/home/arx/arx-competition-control')
for name,src,pkg in [('x5','repos/X5/py/arx_x5_python/bimanual','bimanual'),('lift','repos/LIFT/body/Python/ARX_LIFT_python/lift_controller','lift_controller')]:
    source=r/src;dest=r/'install-python'/name/pkg;dest.mkdir(parents=True,exist_ok=True)
    for p in source.rglob('*'):
        if p.is_file() and p.suffix in ['.py','.urdf'] and not any(x in p.relative_to(source).parts for x in ['api','build','lib','__pycache__']):
            target=dest/p.relative_to(source);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
    with (r/'logs'/('install-python-'+name+'.log')).open('w') as log:
        subprocess.run(['cmake','--install',str(r/'build-python'/name),'--prefix',str(dest/'api')],stdout=log,stderr=subprocess.STDOUT,check=True)
    print('STAGED',name,flush=True)
