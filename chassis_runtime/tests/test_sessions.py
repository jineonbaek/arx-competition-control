import sys,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from arx_chassis.core import Core
from arx_chassis.client import Session
class MemoryTransport:
 def __init__(self):self.outputs=[];self.restart()
 def restart(self):
  self.core=Core(self.outputs.append);self.core.driver=('fake',True,time.monotonic());self.core.bound_instance=self.core.instance
 @property
 def state(self):return SimpleNamespace(scheduler_instance=self.core.instance)
 def manage(self,operation,*,profile='',resume_key='',**kw):
  assert operation=='register'
  return SimpleNamespace(client_id=self.core.register(profile,resume_key),state=self.state)
 def send(self,p,client_id,instance):
  d=dict(p,client_id=client_id,scheduler_instance=instance,kind=p['type'],operation='end' if p['type']=='chassis.end_output' else 'move',sdk=[p.get(k,0.) for k in ('x','y','omega')])
  self.core.command(d);return dict(ok=True)
class SessionTests(unittest.TestCase):
 def payload(self,generation=1):return dict(type='chassis.nav_sdk',source_id='navigation-test',task_id='output',generation=generation,seq=1,issued_at=time.time(),ttl_s=.3,x=.1,y=0.,omega=0.)
 def test_restart_rejects_every_old_generation_until_new_task(self):
  t=MemoryTransport();s=Session('navigation',t);s.send(self.payload());t.restart()
  for seq in range(2,5):
   p=self.payload();p['seq']=seq
   with self.assertRaises(ValueError):s.send(p)
  s.send(self.payload(2));self.assertEqual(t.outputs[-1]['sdk'],[.1,0.,0.])
 def test_web_restart_registration_reuses_principal(self):
  t=MemoryTransport();a=Session('manual',t,resume_key='same-cookie');a.register()
  b=Session('manual',t,resume_key='same-cookie');b.register();self.assertEqual(a.client_id,b.client_id)
 def test_gamepad_initial_end_uses_configured_profile(self):
  root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'navigation/polarbear_adapt'))
  from chassis_command import WebChassis
  transport=MemoryTransport()
  with patch('arx_chassis.client.Session',side_effect=lambda profile:Session(profile,transport)):
   c=WebChassis(source_id='gamepad-test');c.command_kind='chassis.gamepad_sdk';c.sdk_units=True;c.native_sdk=True
   c.begin_task();c.send((.2,0.,0.));self.assertEqual(c.ros_session.profile,'gamepad');self.assertEqual(transport.outputs[-1]['sdk'],[.2,0.,0.])
 def test_maintenance_outage_does_not_escape_observer_tick(self):
  from arx_chassis.proxy import ChassisProxy
  t=MemoryTransport();t.service=SimpleNamespace(service_is_ready=lambda:False)
  with patch('arx_chassis.proxy.shared_transport',return_value=t):p=ChassisProxy()
  p.maintenance=object();started=time.monotonic();p.tick()
  self.assertLess(time.monotonic()-started,.05);self.assertIsNone(p.maintenance)
