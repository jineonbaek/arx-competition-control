"""Integration template only. Explicit callback owns actual operations and evidence."""
import copy,threading,time,uuid
from task_client import TaskClient

def serve(handle_location,*,url='http://127.0.0.1:9985'):
 if not callable(handle_location):raise TypeError('explicit existing task callback required')
 client=TaskClient(url);client.register();stop=threading.Event();failed=[]
 def heartbeat():
  while not stop.wait(.2):
   try:client.heartbeat()
   except Exception as exc:failed.append(exc);return
 thread=threading.Thread(target=heartbeat,daemon=True);thread.start()
 seen=set()
 try:
  while not failed:
   task=client.task()
   if not task or task['status']!='notified' or task['visit_id'] in seen:time.sleep(.1);continue
   seen.add(task['visit_id'])
   # Caller must acquire its existing Controller ownership before accepting.
   # Callback receives an explicit accept function so it chooses that boundary.
   accepted=[False]
   def accept():
    if not accepted[0]:client.event(task,'accept',request_id=str(uuid.uuid4()));accepted[0]=True
   result=handle_location(copy.deepcopy(task),accept)
   if not accepted[0]:raise RuntimeError('task returned without accepting control')
   if (not isinstance(result,dict) or result.get('status')!='completed' or
       result.get('control_released') is not True or result.get('navigation_ready') is not True):
    client.event(task,'fail',request_id=str(uuid.uuid4()),error='task did not supply successful release/readiness evidence');continue
   # Store this request in the caller's journal before retrying uncertain IO.
   request_id=str(uuid.uuid4())
   client.event(task,'complete',request_id=request_id,control_released=True,navigation_ready=True)
 finally:stop.set();thread.join(timeout=1.)

if __name__=='__main__':
 raise SystemExit('Template: import serve and pass your existing task callback. No task was registered or executed.')
