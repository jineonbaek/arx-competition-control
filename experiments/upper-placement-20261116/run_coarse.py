"""Single reusable entry: fresh capture -> plan -> bounded execute -> reobserve.
Default is no-motion planning. Physical execution requires --execute.
"""
import argparse,json,os,subprocess,sys,time
from pathlib import Path

def run_cycles(step,max_cycles):
 if not 1<=max_cycles<=8:raise ValueError('Invalid cycle budget')
 history=[]
 for _ in range(max_cycles):
  result=step();history.append(result);status=result.get('status')
  if status in ('outside_arrived','held','planned'):return dict(status=status,cycles=history)
  if status!='bounded_correction_arrived':raise ValueError('Unknown execution outcome')
 return dict(status='cycle_limit',cycles=history)

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');p.add_argument('--max-cycles',type=int,default=6);a=p.parse_args()
 here=Path(__file__).resolve().parent;work=Path('/home/arx/arx-place-hold-20261116');root=Path('/home/arx/arx-competition-control')
 env=os.environ.copy();env['PYTHONPATH']=os.pathsep.join(map(str,[root,root/'web/src',work/'experiments/placement-handoff-20261116',here]))
 python=str(root/'.venv/bin/python');run=work/'reports'/('d435-coarse-chain-'+time.strftime('%Y%m%d-%H%M%S'));run.mkdir()
 index=0
 def step():
  nonlocal index
  index+=1;cmd=[python,'-u',str(here/'loaded_coarse.py'),'--max-segments','4']
  if a.execute:cmd.append('--execute')
  # Do not kill a child that may own motion. It has bounded waits and its own stop handler.
  child=subprocess.Popen(cmd,cwd=here,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
  lines=[];last=None
  for line in child.stdout:
   print(line,end='',flush=True);lines.append(line)
   try:value=json.loads(line)
   except ValueError:continue
   if 'status' in value:last=value
  code=child.wait();(run/('cycle-%02d.log'%index)).write_text(''.join(lines))
  if code or last is None:return dict(status='held',reason='Child failed; no retry',exit_code=code)
  return last
 result=run_cycles(step,a.max_cycles);(run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(dict(status=result['status'],report=str(run/'result.json'))))
if __name__=='__main__':main()
