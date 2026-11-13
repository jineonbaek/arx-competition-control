"""Build the unpadded external CAD chassis envelope from base and three wheels."""
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import shapely
from scipy.spatial.transform import Rotation

urdf=Path(sys.argv[1]); destination=Path(sys.argv[2])
root=ET.parse(urdf).getroot()
def origin(node):
    t=np.eye(4)
    if node is not None:
        t[:3,3]=np.fromstring(node.get('xyz','0 0 0'),sep=' ')
        t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(node.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return t
base=np.eye(4);base[:3,3]=[.015273,-.000010,.168]
points=[]; selected=('base_link','wheel1','wheel2','wheel3')
for name in selected:
    t=base.copy()
    if name!='base_link':
        joint=next(j for j in root.findall('joint') if j.find('child').get('link')==name)
        if joint.find('parent').get('link')!='base_link':raise ValueError('Unexpected wheel parent')
        t=t@origin(joint.find('origin'))
    link=next(link for link in root.findall('link') if link.get('name')==name)
    for visual in link.findall('visual'):
        mesh=visual.find('geometry/mesh')
        path=urdf.parent.parent/mesh.get('filename').removeprefix('package://lift2/')
        data=path.read_bytes(); count=int.from_bytes(data[80:84],'little')
        if len(data)!=84+50*count:raise ValueError('Expected binary STL')
        dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
        p=np.frombuffer(data,dtype,count=count,offset=84)['vertices'].reshape(-1,3).astype(float)
        p*=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
        transform=t@origin(visual.find('origin'))
        points.append(p@transform[:3,:3].T+transform[:3,3])
shape=shapely.MultiPoint(np.vstack(points)[:,:2]).convex_hull
if not shape.is_valid or shape.area<=0:raise ValueError('Invalid chassis envelope')
destination.parent.mkdir(parents=True,exist_ok=True)
destination.write_text(shapely.to_wkt(shape,rounding_precision=12))
destination.with_suffix('.json').write_text(json.dumps({'source':str(urdf),
    'links':selected,'bounds_m':list(shape.bounds),'area_m2':shape.area,
    'definition':'Unpadded outer convex chassis perimeter; base + three wheels; no arms/grippers',
    'height':'unlimited'},indent=2))
print(json.dumps({'bounds':shape.bounds,'area':shape.area}))
