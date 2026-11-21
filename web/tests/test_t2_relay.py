import unittest,json,tempfile
from pathlib import Path
from navigation_route import RouteMailbox,RouteRelay
class Pub:
 def __init__(self):self.sent=[]
 def get_subscription_count(self):return 1
 def publish(self,v):self.sent.append(v)
class String:
 def __init__(self,data):self.data=data
class RelayTests(unittest.TestCase):
 def test_range_command_keeps_definition_and_rejects_same_id_edit(self):
  with tempfile.TemporaryDirectory() as d:
   definition=dict(route_id='route',points=[dict(id='six',pose=[1.,0.,0.])]);box=RouteMailbox(Path(d),lambda:dict(route=dict(available=True)));r=RouteRelay(Path(d));r.definition=definition;r.available=lambda:True;r.range_pub=Pub();r.cancel_pub=Pub();r.empty=lambda:None;r.string=String
   box.command('start_t2','one','route',expected_definition=definition);r.tick();r.tick();self.assertEqual(len(r.range_pub.sent),1);self.assertEqual(json.loads(r.range_pub.sent[0].data)['expected_definition'],definition)
   box.command('start_t2','stale','route',expected_definition=definition);r.definition=dict(definition,points=[]);r.tick();self.assertEqual(box.receipt('stale')['status'],'rejected');self.assertEqual(len(r.range_pub.sent),1)
   box.command('cancel','stop','wrong');r.tick();self.assertEqual(r.cancel_pub.sent,[])
if __name__=='__main__':unittest.main()
