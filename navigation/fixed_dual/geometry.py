"""Rigid front/rear installation. Metres; matrices map sensor into base_link."""
import json
from pathlib import Path
import numpy as np

def transforms():
    front=np.eye(4);front[:3,3]=[.18533,-.000003,.230]
    rear=np.eye(4);rear[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
    rear[:3,3]=[-.28320,0,.228]
    return front,rear

def compensate(points, body_poses, end_pose, sensor):
    """Each optical point at its own body pose -> optical frame at end_pose."""
    body=np.asarray(points)@sensor[:3,:3].T+sensor[:3,3]
    world=np.einsum('nij,nj->ni',body_poses[:,:3,:3],body)+body_poses[:,:3,3]
    end=end_pose@sensor
    return (world-end[:3,3])@end[:3,:3]

def configuration(root):
    path=Path(root)/'config/fixed_dual.json'
    return json.loads(path.read_text()) if path.exists() else {'enabled':False}

def enabled(root):return bool(configuration(root).get('enabled',False))
