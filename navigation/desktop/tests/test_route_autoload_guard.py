import ast,json,os,unittest
from pathlib import Path
from types import SimpleNamespace as S
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"polarbear_adapt"))
from mission_route import MissionRoute
source=Path(os.environ.get('WAYPOINT_SOURCE',str(Path(__file__).resolve().parents[2]/'polarbear_adapt/waypoint_controls.py')))
method=next(x for c in ast.parse(source.read_text()).body if isinstance(c,ast.ClassDef) and c.name=='Waypoints' for x in c.body if isinstance(x,ast.FunctionDef) and x.name=='edit')
ns={'json':json};exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(source),'exec'),ns)
class Guard(unittest.TestCase):
 def fixtures(self):
  route=MissionRoute('map');new=MissionRoute('map');new.add(dict(type='special',pose=[1,2,0]));obj=S(route=route,driver=S(target=None),blocking=lambda:False,phase=None,show=lambda:None)
  message=dict(action='load_if_empty',definition=new.definition(),expected_route_id=route.route_id)
  return obj,message
 def test_empty_matching_draft_loads(self):
  obj,p=self.fixtures();ns['edit'](obj,S(data=json.dumps(p)));self.assertEqual(obj.route.definition(),p['definition'])
 def test_user_added_point_between_read_and_callback_is_retained(self):
  obj,p=self.fixtures();obj.route.add(dict(type='special',pose=[8,9,0]));before=obj.route.definition();ns['edit'](obj,S(data=json.dumps(p)));self.assertEqual(obj.route.definition(),before)
 def test_explicit_clear_generation_is_retained(self):
  obj,p=self.fixtures();obj.route.reset();before=obj.route.definition();ns['edit'](obj,S(data=json.dumps(p)));self.assertEqual(obj.route.definition(),before)
 def test_active_single_goal_is_retained(self):
  obj,p=self.fixtures();obj.driver.target=[1,0,0];before=obj.route.definition();ns['edit'](obj,S(data=json.dumps(p)));self.assertEqual(obj.route.definition(),before)
if __name__=='__main__':unittest.main()
