import tempfile,unittest,json,uuid
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from competition_route import coordinate_id,route_payload,select_payload,read_definition
class RouteStore(unittest.TestCase):
 def definition(self,map_id='live:run',points=True):
  return dict(schema_version=2,route_id=str(uuid.uuid4()),map_id=map_id,points=[dict(id=str(uuid.uuid4()),type='special',pose=[1,2,.3])] if points else [])
 def test_coordinate_inherited_on_resume_but_changed_for_new_heading(self):
  with tempfile.TemporaryDirectory() as t:
   root=Path(t);old=root/'old';old.mkdir();(old/'coordinate-frame.json').write_text(json.dumps(dict(coordinate_id='old-frame')))
   run=root/'reports/run';run.mkdir(parents=True);self.assertEqual(coordinate_id(root,run,old/'room.yaml'),'old-frame')
   run2=root/'reports/new';run2.mkdir();self.assertNotEqual(coordinate_id(root,run2,old/'room.yaml',True),'old-frame')
 def test_same_frame_rebinds_only_map_id_preserves_points(self):
  d=self.definition();p=route_payload(d,'frame')
  with tempfile.TemporaryDirectory() as t:
   f=Path(t);(f/'competition-route.json').write_text(json.dumps(p));loaded=read_definition(f,'frame','live:new')
  self.assertEqual(loaded['points'],d['points']);self.assertEqual(loaded['map_id'],'live:new')
 def test_wrong_frame_cannot_load_route(self):
  with tempfile.TemporaryDirectory() as t:
   f=Path(t);(f/'competition-route.json').write_text(json.dumps(route_payload(self.definition(),'old')))
   with self.assertRaises(ValueError):read_definition(f,'new','live:run')
 def test_explicit_clear_saves_empty_instead_of_old_route(self):
  old=route_payload(self.definition(),'same');new=self.definition(points=False)
  self.assertEqual(select_payload(new,old,'same')['definition']['points'],[])
 def test_mapping_preserves_route_only_for_same_coordinates(self):
  old=route_payload(self.definition(),'old');self.assertEqual(select_payload(None,old,'old'),old);self.assertIsNone(select_payload(None,old,'new'))
if __name__=='__main__':unittest.main()
