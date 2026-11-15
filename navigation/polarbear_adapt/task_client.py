"""Task caller client. No agent edits, motion commands or automatic task completion."""
import json,uuid
from urllib.request import Request,build_opener,ProxyHandler
from urllib.parse import urlsplit

class TaskClient:
 def __init__(self,url='http://127.0.0.1:9985'):
  parsed=urlsplit(url)
  if parsed.scheme!='http' or parsed.hostname not in ('127.0.0.1','localhost') or parsed.path not in ('','/'):raise ValueError('local navigation API required')
  self.url=url.rstrip('/');self.token=None;self.instance_id=str(uuid.uuid4());self.opener=build_opener(ProxyHandler({}))
 def _request(self,path,value=None,auth=True):
  headers={}
  if auth and self.token:headers['Authorization']='Bearer '+self.token
  raw=None if value is None else json.dumps(value,allow_nan=False).encode()
  if raw is not None:headers['Content-Type']='application/json'
  with self.opener.open(Request(self.url+path,raw,headers),timeout=.3) as reply:return json.load(reply)
 def register(self,point_ids=('point_1','point_2')):
  result=self._request('/v1/worker',dict(worker_id='battery',instance_id=self.instance_id,point_ids=list(point_ids)),auth=False);self.token=result['token'];return result
 def heartbeat(self):return self._request('/v1/heartbeat',{})
 def state(self):return self._request('/v1/state',auth=False)
 def task(self):return self._request('/v1/task')['task']
 def event(self,task,event,*,request_id,**result):
  data={k:task[k] for k in ('mission_id','visit_id','point_id')}
  data.update(request_id=request_id,event=event,**result)
  return self._request('/v1/task',data)

if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser(description='Read navigation handoff state only; does not register or complete tasks.')
 p.add_argument('--url',default='http://127.0.0.1:9985');args=p.parse_args()
 print(json.dumps(TaskClient(args.url).state(),ensure_ascii=False,indent=2))
