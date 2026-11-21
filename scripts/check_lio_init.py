"""Last-boundary lease/state check; never creates or changes motion targets."""
import os,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'chassis_runtime'))
from arx_chassis.client import Session
from navigation_services import robot_idle
s=Session('manual');s.client_id=os.environ['ARX_LIO_INIT_CLIENT'];s.instance=os.environ['ARX_LIO_INIT_INSTANCE']
# Talk with the exact existing session; do not auto-register across scheduler restart.
s.transport.manage('heartbeat',client_id=s.client_id,instance=s.instance)
assert robot_idle(),'base state changed before LIO launch'
