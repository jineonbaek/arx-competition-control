import contextlib,importlib,io,json,tempfile,unittest
from pathlib import Path
from mission_route import MissionRoute

class RouteCheck(unittest.TestCase):
 def test_saved_definition_checks_task_identity_without_starting_or_editing(self):
  checker=importlib.import_module('check_task_route')
  with tempfile.TemporaryDirectory() as tmp:
   r=MissionRoute('saved-room');r.add(dict(type='task',point_id='point_1',pose=[1,.3,.7]));r.add(dict(type='task',point_id='point_2',pose=[2,.8,-.4]));p=Path(tmp)/'route.json';p.write_text(json.dumps(r.definition()));before=p.read_bytes();out=io.StringIO()
   with contextlib.redirect_stdout(out):code=checker.main(['--file',str(p)])
   v=json.loads(out.getvalue());self.assertEqual(code,0);self.assertEqual(v['route']['task_points'][1]['pose'],[2.,.8,-.4]);self.assertIsNone(r.mission_id);self.assertEqual(p.read_bytes(),before)
   v=r.definition();v['points'][1]['point_id']='point_1';p.write_text(json.dumps(v))
   with contextlib.redirect_stdout(io.StringIO()):code=checker.main(['--file',str(p)])
   self.assertEqual(code,2)
if __name__=='__main__':unittest.main()
