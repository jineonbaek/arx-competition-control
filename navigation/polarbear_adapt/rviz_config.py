from pathlib import Path
import os
import yaml
p=Path(__file__).resolve().parent

def topic(value,transient=False,best=False):return {'Value':value,'Reliability Policy':'Best Effort' if best else 'Reliable','Durability Policy':'Transient Local' if transient else 'Volatile','Depth':2}
displays=[]
for name,t,color,enabled,decay in [('Bottom scan','/single/cloud_registered','0; 255; 120',True,0),
 ('Bottom obstacle observations','/pb/bottom/nav_mark','255; 130; 0',False,.4),
 ('Head obstacle observations (parked)','/pb/head/nav_mark','255; 45; 45',True,.4)]:
 displays.append({'Class':'rviz_default_plugins/PointCloud2','Name':name,'Enabled':enabled,'Topic':topic(t,best=True),'Size (m)':.025,'Style':'Points','Color Transformer':'FlatColor','Color':color,'Decay Time':decay})
for name,t in [('Head scan (parked)','/pb/head_scene'),]:
 displays.append({'Class':'rviz_default_plugins/Marker','Name':name,'Enabled':True,'Topic':topic(t)})
import sys
sys.path.insert(0,str(p.parent/'fixed_dual'))
from geometry import enabled as fixed_enabled
if fixed_enabled(p.parent):
 displays=[v for v in displays if v['Name']!='Head scan (parked)']
 for v in displays:
  if v['Name']=='Bottom scan':v.update(Name='双雷达联合 LIO 点云',Topic=topic('/pb/rviz/lio_cloud_map',best=True))
  if v['Name']=='Head obstacle observations (parked)':v.update(Name='后雷达障碍观测')
 displays.append({'Class':'rviz_default_plugins/PointCloud2','Name':'后雷达去畸变点云','Enabled':True,'Topic':topic('/pb/rviz/rear_cloud_map',best=True),'Size (m)':.025,'Style':'Points','Color Transformer':'FlatColor','Color':'0; 255; 0','Decay Time':0})
for key,name,enabled in [('map','Map',True),('global','Global costmap',True),('local','Local costmap',False)]:
 displays.append({'Class':'rviz_default_plugins/MarkerArray','Name':name,'Enabled':enabled,'Topic':topic('/pb/display/'+key,True)})
for priority,(name,t,col) in enumerate([('Actual CAD footprint','/pb/rviz/footprint','255; 255; 0'),('Padded CAD planner footprint','/pb/rviz/padded_footprint','255; 0; 255')],1):
 displays.append({'Class':'arx_waypoints/ForegroundPolygon','Name':name,'Enabled':True,'Topic':topic(t,transient=True),'Color':col,'Alpha':1.,'Draw Priority':priority,'Frame Locked':True})
displays.append({'Class':'rviz_default_plugins/Path','Name':'Planned path','Enabled':True,'Topic':topic('/pb/path'),'Color':'0; 80; 255','Line Style':'Billboards','Line Width':.04,'Offset':{'X':0.,'Y':0.,'Z':.12}})
displays.append({'Class':'rviz_default_plugins/Marker','Name':'First path issue (amber unknown / red obstacle)','Enabled':True,'Topic':topic('/pb/path_issue')})
x={'Panels':[{'Class':'rviz_common/Displays','Name':'Displays'}],'Visualization Manager':{'Class':'','Global Options':{'Fixed Frame':'map','Background Color':'45; 45; 45','Frame Rate':20},'Displays':displays,'Tools':[{'Class':'rviz_default_plugins/Interact'},{'Class':'rviz_default_plugins/MoveCamera'},{'Class':'rviz_default_plugins/Select'},{'Class':'rviz_default_plugins/SetGoal','Topic':'/pb/plan_goal'},{'Class':'rviz_default_plugins/PublishPoint','Topic':'/clicked_point','Single click':True}],'Views':{'Current':{'Class':'rviz_default_plugins/TopDownOrtho','Scale':65,'X':0.,'Y':0.,'Angle':-1.5707963267948966}}},'Window Geometry':{'Width':1400,'Height':900}}
if os.getenv('ARX_DESKTOP_MODE') in ('mapping','mapping-navigation'):
 x['Panels'].append({'Class':'arx_waypoints/MappingPanel','Name':'建图操作'})
if os.getenv('ARX_DESKTOP_MODE')!='mapping':
 x['Panels'].append({'Class':'arx_waypoints/Panel','Name':'多点导航'})
 x['Visualization Manager']['Tools'].append({'Class':'arx_waypoints/Add'})
 displays.append({'Class':'rviz_default_plugins/MarkerArray','Name':'有序航点','Enabled':True,'Topic':topic('/pb/waypoints/markers',True)})
(p/'polarbear.rviz').write_text(yaml.safe_dump(x,sort_keys=False))
