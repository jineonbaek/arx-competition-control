"""Suspend execution time only while output is parked, including nested waits."""
import time
from contextlib import contextmanager
from functools import wraps

def paused_budget(method):
 @wraps(method)
 def wrapped(driver,*args,**kwargs):
  outer=getattr(driver,'_budget_wait_depth',0)==0
  identity=driver.seq
  began=time.monotonic();driver._budget_wait_depth=getattr(driver,'_budget_wait_depth',0)+1
  if outer:driver._budget_pause=(identity,began)
  try:return method(driver,*args,**kwargs)
  finally:
   driver._budget_wait_depth-=1
   if outer and driver.seq==identity:driver.started+=time.monotonic()-began
   if outer:driver._budget_pause=None
 return wrapped

@contextmanager
def charged_motion(driver):
 """An escape performed inside a parked replan still spends execution time."""
 identity=driver.seq
 pause=getattr(driver,'_budget_pause',None)
 charging=bool(pause and pause[0]==identity);began=time.monotonic()
 previous=getattr(driver,'_budget_charge',None)
 if charging:driver._budget_charge=(identity,began)
 try:yield
 finally:
  if charging and driver.seq==identity:driver.started-=time.monotonic()-began
  driver._budget_charge=previous

def require_execution_budget(driver):
 now=time.monotonic();elapsed=now-driver.started
 pause=getattr(driver,'_budget_pause',None)
 if pause and pause[0]==driver.seq:
  elapsed-=now-pause[1]
  charge=getattr(driver,'_budget_charge',None)
  if charge and charge[0]==driver.seq:elapsed+=now-charge[1]
 if elapsed>=480:raise ValueError('goal_timeout')
