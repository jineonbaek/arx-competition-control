from pathlib import Path
import yaml
p=Path(__file__).resolve().parent

def topic(value,transient=False,best=False):return {'Value':value,'Reliability Policy':'Best Effort' if best else 'Reliable','Durability Policy':'Transient Local' if transient else 'Volatile','Depth':2}
displays=[]
for name,t,color,enabled in [('Current scan','/single/cloud_registered','0; 255; 120',True),('Ground','/preview/ground','120; 120; 120',False),('Obstacles','/preview/obstacles','255; 80; 0',True)]:
 displays.append({'Class':'rviz_default_plugins/PointCloud2','Name':name,'Enabled':enabled,'Topic':topic(t,best=True),'Size (m)':.025,'Style':'Points','Color Transformer':'FlatColor','Color':color,'Decay Time':0})
for name,t,alpha,enabled,scheme in [('Live SLAM grid','/preview/slam_map',.8,True,'map'),('LIO projected grid comparison','/mapping/grid',.6,False,'map'),('Global costmap','/global_costmap/costmap',.6,True,'costmap'),('Local costmap','/local_costmap/costmap',.5,False,'costmap')]:
 displays.append({'Class':'rviz_default_plugins/Map','Name':name,'Enabled':False,'Topic':topic(t,True),'Alpha':alpha,'Color Scheme':scheme})
for key,name,enabled in [('map','Live SLAM grid (compatible display)',True),('global','Global costmap (compatible display)',True),('local','Local costmap (compatible display)',False),('projection','LIO projection (compatible display)',False)]:
 displays.append({'Class':'rviz_default_plugins/MarkerArray','Name':name,'Enabled':enabled,'Topic':topic('/preview/display/'+key,True)})
for name,t,col in [('Actual CAD footprint','/preview/footprint','255; 255; 0'),('Conservative planner footprint','/global_costmap/published_footprint','255; 0; 255')]:displays.append({'Class':'rviz_default_plugins/Polygon','Name':name,'Enabled':True,'Topic':topic(t),'Color':col,'Alpha':1.})
displays.append({'Class':'rviz_default_plugins/Path','Name':'Plan ONLY - no execution','Enabled':True,'Topic':topic('/preview/path'),'Color':'0; 80; 255','Line Style':'Lines','Line Width':.04})
x={'Panels':[{'Class':'rviz_common/Displays','Name':'Displays'}],'Visualization Manager':{'Class':'','Global Options':{'Fixed Frame':'preview_map','Background Color':'45; 45; 45','Frame Rate':20},'Displays':displays,'Tools':[{'Class':'rviz_default_plugins/Interact'},{'Class':'rviz_default_plugins/MoveCamera'},{'Class':'rviz_default_plugins/Select'},{'Class':'rviz_default_plugins/PublishPoint','Topic':'/clicked_point','Single click':True}],'Views':{'Current':{'Class':'rviz_default_plugins/TopDownOrtho','Scale':65,'X':0.,'Y':0.,'Angle':-1.5707963267948966}}},'Window Geometry':{'Width':1400,'Height':900}}
(p/'preview.rviz').write_text(yaml.safe_dump(x,sort_keys=False))
