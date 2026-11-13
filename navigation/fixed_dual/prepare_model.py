"""Move only the upper MID360 mesh; retain the mast/cameras at their existing poses."""
import json,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from geometry import transforms
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'dual_flex'))
from mesh_filter import origin_transform,urdf_geometry,voxelize_triangles
source=ROOT/'models/lift2s_bottom_upright_20261105'
target=ROOT/'models/lift2s_fixed_rear_20261112'
(target/'lift2/urdf').mkdir(parents=True,exist_ok=True)
if not (target/'lift2/meshes').exists():(target/'lift2/meshes').symlink_to((source/'lift2/meshes').resolve(),target_is_directory=True)
cfg=json.loads((ROOT/'config/bottom_upright_20261105.json').read_text())
tree=ET.parse(source/'lift2/urdf/lift2_with_sensors.urdf');joints={j.get('name'):j for j in tree.getroot().findall('joint')}
body_from_base=np.eye(4);body_from_base[:3,3]=-np.array(cfg['frame']['origin_in_urdf_base_link_m'])
old_mesh=body_from_base@origin_transform(joints['base_to_head_113_fixed'].find('origin'))@origin_transform(joints['head_113_to_upper_mid360_fixed'].find('origin'))
old=np.eye(4);old[:3,:3]=np.diag([1,-1,-1]);old[:3,3]=cfg['top']['lidar_origin_m']
front,rear=transforms();new_mesh=np.linalg.inv(body_from_base)@rear@np.linalg.inv(old)@old_mesh
j=joints['head_113_to_upper_mid360_fixed'];j.set('name','base_to_rear_mid360_fixed');j.find('parent').set('link','base_link')
j.find('origin').set('xyz',' '.join(map(str,new_mesh[:3,3])));j.find('origin').set('rpy',' '.join(map(str,Rotation.from_matrix(new_mesh[:3,:3]).as_euler('xyz'))))
urdf=target/'lift2/urdf/lift2_with_sensors.urdf';tree.write(urdf,encoding='utf-8',xml_declaration=True)
parts=urdf_geometry(urdf,cfg['frame']['origin_in_urdf_base_link_m'])
with np.load(source/'body_voxels.npz') as z:
    rigid=z['rigid'];nominal=z['nominal_head'];pitch=float(z['pitch'])
    # All other rigid mesh samples remain byte-for-byte; only rebuild former head group.
    remaining=[p for p in parts if p['head'] or p['link']=='upper_mid360_link']
    head=np.unique(np.vstack([voxelize_triangles(p['triangles'],pitch) for p in remaining]),axis=0)
    np.savez_compressed(target/'body_voxels.npz',rigid=rigid,head=head,nominal_head=nominal,pitch=pitch)
import shutil
shutil.copy2(source/'base_interior.npz',target/'base_interior.npz')
rear_imu=rear[:3,3]+rear[:3,:3]@np.array([.011,.02329,-.04412])
data=dict(enabled=False,mode='fixed_front_rear',source='user installation 2026-11-12; nominal, not precision calibrated',
          mesh=str((target/'body_voxels.npz').relative_to(ROOT)),front_in_base=front.tolist(),rear_in_base=rear.tolist(),
          rear_mount_m=[-.2832,0,.181],rear_imu_m=rear_imu.tolist(),front_from_rear=(np.linalg.inv(front)@rear).tolist(),
          optical_height_m={'bottom':.230,'head':.228},identity={'bottom':'192.168.1.3','head':'192.168.1.164'},
          model_note='Only upper_mid360_link relocated; mast and cameras retained; new brackets/cables not modeled')
path=ROOT/'config/fixed_dual.json'
if path.exists():data['enabled']=json.loads(path.read_text()).get('enabled',False)
path.write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps(data,indent=2))
