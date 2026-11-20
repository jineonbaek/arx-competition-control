"""Zero-only browser bridge wire-format test, including JSON integer TTL."""
import json,time,uuid,urllib.request,http.cookiejar
from pathlib import Path
origin='http://127.0.0.1:9983';jar=http.cookiejar.CookieJar();client=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(jar),urllib.request.ProxyHandler({}))
client.open(origin+'/',timeout=3).read()
source='web-http-check-'+uuid.uuid4().hex[:12]
def send(kind,seq):
 p=dict(type=kind,source_id=source,task_id='keyboard',generation=1,seq=seq,issued_at=time.time(),ttl_s=1)
 if kind=='chassis':p.update(x=0,y=0,omega=0)
 req=urllib.request.Request(origin+'/api/command',json.dumps(p).encode(),{'Origin':origin,'Content-Type':'application/json'})
 with client.open(req,timeout=6) as r:d=json.load(r)
 assert d['ok'],d;return d
move=send('chassis',1);end=send('chassis.end_output',2)
state=json.load(client.open(origin+'/api/state',timeout=3));assert not state['chassis_active'] and state['chassis_scheduler']['active'] is None
result=dict(browser_integer_ttl_accepted=True,normal_release_accepted=True,move_output_seq=move.get('output_seq'),end_output_seq=end.get('output_seq'),chassis_active=state['chassis_active'])
Path('/home/arx/arx-competition-control/navigation/reports/ros-chassis-20261120/http-verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
