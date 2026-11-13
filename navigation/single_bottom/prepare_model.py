"""Version new URDF without modifying the inverted historical model."""
import json,sys,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from core import upright_config,updated_lower_mesh_transform
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'dual_flex'))
from mesh_filter import origin_transform,build
source=root/'models/lift2s_20261105';target=root/'models/lift2s_bottom_upright_20261105'
(target/'lift2/urdf').mkdir(parents=True,exist_ok=True)
if not (target/'lift2/meshes').exists():(target/'lift2/meshes').symlink_to(source/'lift2/meshes',target_is_directory=True)
cad=json.loads((root/'config/cad_nominal_20261105.json').read_text());old=cad['bottom'].copy();cad['bottom'].update(upright_config());cad['status']='current_bottom_single_upright';cad['source']='user confirmed upright 183mm mounting surface, x moved rearward 12mm; y unchanged';cad['applicable_recordings']=[];cad['relative']['dynamic_relative_pose_fixed']=False;cad['relative']['note']='Historical equal-orientation translations invalid for current upright bottom';
for k in ['head_from_bottom_translation_m','bottom_from_head_translation_m','vehicle_frame_head_minus_bottom_m']:cad['relative'].pop(k,None)
cad['runtime_apply_blockers']=['measurement uncertainty not independently calibrated','head still flexible; not used for single LIO'];cad['removed_geometry']=['front_hang_mount_link'];cad['unmodeled_geometry']='new mounting plate/cables not supplied'
config=root/'config/bottom_upright_20261105.json';config.write_text(json.dumps(cad,indent=2))
xml=ET.parse(source/'lift2/urdf/lift2_with_sensors.urdf');tree=xml.getroot();joints={j.get('name'):j for j in tree.findall('joint')}
old_base_mesh=origin_transform(joints['base_to_front_hang_mount_fixed'].find('origin'))@origin_transform(joints['front_hang_mount_to_lower_mid360_fixed'].find('origin'))
body_from_base=np.eye(4);body_from_base[:3,3]=-np.array(cad['frame']['origin_in_urdf_base_link_m']);old_lidar=np.eye(4);old_lidar[:3,:3]=np.diag([1,-1,-1]);old_lidar[:3,3]=old['lidar_origin_m'];new_base_mesh=np.linalg.inv(body_from_base)@updated_lower_mesh_transform(body_from_base@old_base_mesh,old_lidar)
tree.remove(joints['base_to_front_hang_mount_fixed']);tree.remove(next(x for x in tree.findall('link') if x.get('name')=='front_hang_mount_link'))
j=joints['front_hang_mount_to_lower_mid360_fixed'];j.set('name','base_to_lower_upright_fixed');j.find('parent').set('link','base_link');o=j.find('origin');o.set('xyz',' '.join(map(str,new_base_mesh[:3,3])));o.set('rpy',' '.join(map(str,Rotation.from_matrix(new_base_mesh[:3,:3]).as_euler('xyz'))))
urdf=target/'lift2/urdf/lift2_with_sensors.urdf';xml.write(urdf,encoding='utf-8',xml_declaration=True)
build(urdf,config,target/'body_voxels.npz')
(root/'config/current_bottom_sensor.json').write_text(json.dumps({'configuration':str(config),'mesh':str(target/'body_voxels.npz'),'mode':'single_bottom_upright','motion_commands_allowed':False},indent=2))
