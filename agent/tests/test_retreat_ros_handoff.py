"""Exercise the real ROS session protocol with inert output, without ROS or hardware."""
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch

from agent.tests import test_chassis_retreat as retreat_tests

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'chassis_runtime'))
from arx_chassis.core import Core
from arx_chassis.proxy import ChassisProxy
from navigation.polarbear_adapt.chassis_command import WebChassis
from web.src.navigation_session import NavigationSession


class ColdTransport:
    def __init__(self):
        self.wall=1000.;self.outputs=[];self.messages=[]
        self.core=Core(self.outputs.append,wall=lambda:self.wall,clock=lambda:50.)
        self.core.driver=('inert',True,50.);self.core.bound_instance=self.core.instance

    @property
    def state(self):return SimpleNamespace(scheduler_instance=self.core.instance)

    def manage(self,operation,*,profile='',resume_key='',**kwargs):
        assert operation=='register'
        self.wall+=1.  # DDS discovery outlives the existing 300 ms navigation lease.
        return SimpleNamespace(client_id=self.core.register(profile,resume_key),state=self.state)

    def send(self,payload,client_id,instance):
        self.messages.append(payload.copy())
        self.core.command(dict(payload,client_id=client_id,scheduler_instance=instance,
            kind=payload['type'],operation='end' if payload['type']=='chassis.end_output' else 'move',
            sdk=[payload.get(k,0.) for k in ('x','y','omega')]))
        return dict(ok=True)


class RetreatRosTests(unittest.TestCase):
    def test_cold_registration_precedes_first_retreat_timestamp(self):
        fixture=retreat_tests.RetreatTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        transport=ColdTransport()
        with patch('arx_chassis.proxy.shared_transport',return_value=transport):proxy=ChassisProxy()
        controller=fixture.controller;controller.config={'chassis_backend':'ros'};controller.chassis=proxy
        def submit(command,token):
            sdk=None if command['type']=='chassis.end_output' else tuple(command[k] for k in ('x','y','omega'))
            return proxy.submit(command,token,sdk)
        controller.submit=submit
        with patch('agent.chassis_retreat.WebChassis',side_effect=lambda **kw:WebChassis(wall=lambda:transport.wall,**kw)):
            fixture.step.start()
            self.assertEqual(transport.outputs,[])
            fixture.step.poll()
            self.assertLess(transport.outputs[-1]['sdk'][0],0.)
            self.assertEqual(transport.messages[0]['issued_at'],1001.)
            self.assertEqual(transport.messages[0]['ttl_s'],.3)
            fixture.step.cancel()
        self.assertEqual(transport.messages[-1]['type'],'chassis.end_output')
        self.assertTrue(all(value['sdk']==[0.,0.,0.] for value in transport.outputs[-1:]))
        self.assertEqual(len(proxy.sessions),1)

    def test_navigation_complete_waits_for_release_and_reuses_existing_visit(self):
        client=Mock();controller=Mock()
        controller.snapshot.side_effect=[dict(chassis_active=False,motion_reserved=True),
                                        dict(chassis_active=False,motion_reserved=False)]
        session=NavigationSession(controller,Mock(),client)
        session.active=True;session.context={'run_id':'competition-test','cancelled':threading.Event()}
        session.visit=dict(mission_id='mission',visit_id='visit',point_id='point_1')
        session.complete_visit(session.context)
        client.event.assert_called_once_with(session.visit,'complete',
            request_id=session.events[('visit','complete')][0],control_released=True,navigation_ready=True)


if __name__=='__main__':unittest.main()
