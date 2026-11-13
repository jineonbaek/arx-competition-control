"""Pure geometry for upright bottom MID360S; all coordinates are metres."""
import numpy as np
from scipy.spatial.transform import Rotation

def upright_config():
 return {'mount_center_m':[.185330,-.000003,.183], 'lidar_origin_m':[.185330,-.000003,.230], 'imu_origin_m':[.196330,.023287,.185880], 'rpy_rad':[0.,0.,0.], 'source':'user measured installation; not precision calibration', 'ip':'192.168.1.3', 'serial':'ARMCPA80036301'}

def updated_lower_mesh_transform(old_mesh_in_body, old_lidar_in_body):
 new=np.eye(4);new[:3,3]=upright_config()['lidar_origin_m']
 return new@np.linalg.inv(old_lidar_in_body)@old_mesh_in_body

def skew(p):
 x,y,z=p;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])

def body_state(position,quaternion,twist,imu_origin):
 R=Rotation.from_quat(quaternion).as_matrix();t=np.asarray(imu_origin);p=np.asarray(position)-R@t
 J=np.eye(6);J[:3,3:]=skew(t)
 Jp=np.eye(6);Jp[:3,3:]=skew(R@t)
 return p,Rotation.from_matrix(R).as_quat(),J@twist,J,Jp

def initial_gauge(body_pose):
 # Keep gravity-aligned vertical, set initial heading and origin to zero.
 yaw=np.arctan2(body_pose[1,0],body_pose[0,0]);G=np.eye(4);G[:3,:3]=Rotation.from_euler('z',-yaw).as_matrix();G[:3,3]=-G[:3,:3]@body_pose[:3,3];return G

def localization_initial_pose(env):
 try:values=[float(env[k]) for k in ['ARX_INITIAL_X','ARX_INITIAL_Y','ARX_INITIAL_YAW']]
 except (KeyError,ValueError) as exc:raise ValueError('Explicit saved-map initial X, Y and yaw are required') from exc
 if not np.isfinite(values).all():raise ValueError('Initial pose must be finite')
 return values

def localization_seed_covariance():
 # Initial search spread, not a calibrated posterior accuracy claim.
 C=np.zeros((6,6));C[0,0]=C[1,1]=.1**2;C[5,5]=np.deg2rad(10.)**2;return C.ravel().tolist()
