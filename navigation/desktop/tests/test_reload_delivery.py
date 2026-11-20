import copy,json,tempfile,unittest,sys
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import competition_route as m
class Delivery(unittest.TestCase):
 def setup_transport(self,drop=0,concurrent=False):
  self.clock=0.;self.calls=[];self.drop=drop;self.concurrent=concurrent
  self.initial=dict(schema_version=2,route_id='initial',map_id='live:run',points=[])
  self.desired=dict(self.initial,route_id='saved',points=[{'id':'p','type':'special','pose':[1,2,0]}]);self.current=copy.deepcopy(self.initial);self.callbacks={}
  def spin(*args,**kw):
   self.clock+=.05
   if concurrent and self.calls:self.current=dict(self.initial,route_id='user-edit')
  def publish(msg):
   data=json.loads(msg.data);self.calls.append((self.clock,data))
   if len(self.calls)<=drop:return
   if data.get('expected_definition',self.initial)==self.current:
    self.current=copy.deepcopy(self.desired)
    cb=self.callbacks.get('/pb/waypoints/definition')
    if cb:cb(S(data=json.dumps(self.current)))
  pub=S(publish=publish,get_subscription_count=lambda:1)
  def subscribe(kind,topic,cb,*args):self.callbacks[topic]=cb;return S()
  def endpoints(topic):return [S(node_name='_NODE_NAME_UNKNOWN_',node_namespace='/')]+([S(node_name='pb_follow_once',node_namespace='/')] if self.clock>=.3 else [])
  self.node=S(create_publisher=lambda *a:pub,create_subscription=subscribe,get_subscriptions_info_by_topic=endpoints,destroy_node=lambda:None)
  ros=S(init=lambda:None,shutdown=lambda:None,create_node=lambda *a:self.node,spin_once=spin)
  qos=S(QoSProfile=lambda **kw:kw,DurabilityPolicy=S(TRANSIENT_LOCAL=1))
  return {'rclpy':ros,'rclpy.qos':qos,'std_msgs.msg':S(String=lambda **kw:S(**kw))}
 def run_load(self,**kw):
  modules=self.setup_transport(**kw)
  with tempfile.TemporaryDirectory() as t:
   path=Path(t);(path/'competition-route.json').write_text('{}')
   with patch.dict(sys.modules,modules),patch.object(m,'validate'),patch.object(m,'coordinate_id',return_value='frame'),patch.object(m,'read_definition',return_value=self.desired),patch.object(m,'observer',return_value=lambda:{'route':{'available':True,'status':'待开始 0/0','drive_status':'ready_for_new_rviz_arrow','definition':copy.deepcopy(self.current)},'competition':{}}),patch.object(m.time,'monotonic',side_effect=lambda:self.clock):
    # read_definition adapts the route map identity to this run in production.
    self.initial['map_id']=self.current['map_id']=self.desired['map_id']='live:'+str(path)
    m.load(path,path,path/'room.yaml',replace=True)
 def test_first_packet_loss_retries_same_guarded_edit(self):
  self.run_load(drop=1);self.assertEqual(self.current,self.desired);self.assertGreaterEqual(len(self.calls),2)
  self.assertTrue(all(p['expected_definition']==self.initial for _,p in self.calls));self.assertTrue(all(p['action']=='replace_if_unchanged' for _,p in self.calls))
 def test_unknown_subscriber_cannot_trigger_early_publish(self):
  self.run_load();self.assertGreaterEqual(self.calls[0][0],.3)
 def test_user_edit_during_retry_is_preserved_and_reported(self):
  with self.assertRaisesRegex(ValueError,'路线已改变'):self.run_load(drop=100,concurrent=True)
  self.assertEqual(self.current['route_id'],'user-edit');self.assertEqual(len(self.calls),1)
if __name__=='__main__':unittest.main()
