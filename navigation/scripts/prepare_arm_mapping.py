#!/usr/bin/env python3
from pathlib import Path
import datetime,json,subprocess,yaml,os
root=Path(__file__).resolve().parents[1]
if subprocess.run(['ping','-I','enp130s0','-c','1','-W','1','192.168.1.3'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:
 raise SystemExit('Configured arm MID360S at 192.168.1.3 is not reachable')
addresses=json.loads(subprocess.check_output(['ip','-j','addr','show','dev','enp130s0'],text=True))
if not any(a.get('local')=='192.168.1.50' for a in addresses[0]['addr_info']):raise SystemExit('Missing temporary sensor receive IPv4 192.168.1.50')
run=root/'reports'/datetime.datetime.now().strftime('arm-lidar-mapping-%Y%m%d-%H%M%S');run.mkdir(parents=True)
p=yaml.safe_load((root/'config/mapping_test/fastlio_arm.yaml').read_text());p['/**']['ros__parameters']['map_file_path']=str(run/'map.pcd');(run/'fastlio.yaml').write_text(yaml.safe_dump(p,sort_keys=False));(root/'reports/latest-arm-mapping-path.txt').write_text(str(run)+'\n');link=root/'maps/arm_lidar_latest.pcd'
if link.exists() and not link.is_symlink():raise SystemExit('Refusing to replace a regular map file')
tmp=link.with_suffix('.pcd.link');tmp.symlink_to(run/'map.pcd');os.replace(tmp,link)
print(run)
