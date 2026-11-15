"""Frozen-image Qwen comparison only. No ROS, SDK, camera or motion access."""
import sys,json,time,hashlib
from pathlib import Path
from PIL import Image
WORK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WORK/'vision/demo'))
from pipeline import QwenJudge,overlay_mask,overlay_candidates
from target import load_reference,shape_view,TASK
OUT=Path(__file__).resolve().parent
class ShortJudge(QwenJudge):
    def __init__(self):
        super().__init__(Path('/home/arx/arx-competition-control/agent/.secrets/qwen.env'))
        self.client.config.profiles['cloud']['enable_thinking']=False
        self.client.config.max_output_tokens=512
    def ask(self,text,images,properties,required):
        properties=dict(properties,confidence={'type':'number','minimum':0,'maximum':1})
        result=super().ask(text+' Return a concise decision, confidence from 0 to 1, and one short reason (at most 30 Chinese characters). Confidence is recorded only; no confidence acceptance threshold. Do not provide a long analysis.',images,properties,[*required,'confidence'])
        assert type(result['confidence']) in (int,float) and 0<=result['confidence']<=1
        return result

def main():
    cases=json.loads((OUT/'cases.json').read_text())
    reference=shape_view(load_reference())
    judge=ShortJudge()
    assert judge.client.config.profiles['cloud']['enable_thinking'] is False
    assert judge.client.config.max_output_tokens==512
    (OUT/'settings.json').write_text(json.dumps({'model':judge.client.config.profiles['cloud']['model'],'enable_thinking':False,'max_output_tokens':512,'confidence_gate':False,'baseline':'saved historical requests; not simultaneous A/B','inputs':'original candidates, original mask, original grayscale preprocessing','execution':'sequential cloud requests only'},indent=2))
    summary=[]
    for index,case in enumerate(cases,1):
        directory=Path(case['directory'])
        image=shape_view(Image.open(directory/'rgb.png').convert('RGB'))
        mask=Image.open(directory/'mask.png').convert('L')
        import numpy as np
        overlay=overlay_mask(image,np.asarray(mask)>0)
        record={'index':index,'case_id':case['case_id'],'old':case['old'],'new':{}}
        for task in ['selection','review']:
            started=time.monotonic();before=len(judge.records)
            try:
                result=(judge.select(image,reference,case['candidates'],TASK) if task=='selection' else judge.review(image,reference,overlay,case['geometry'],TASK))
                report=dict(judge.records[-1]);report['wall_seconds']=round(time.monotonic()-started,3)
                record['new'][task]=report
                print(json.dumps({'index':index,'task':task,'seconds':report['seconds'],'result':result},ensure_ascii=False),flush=True)
            except Exception as error:
                record['new'][task]={'seconds':round(time.monotonic()-started,3),'error':str(error),'result':None}
                print(json.dumps({'index':index,'task':task,'error':str(error)},ensure_ascii=False),flush=True)
            (OUT/f'case-{index:02d}.json').write_text(json.dumps(record,ensure_ascii=False,indent=2))
        summary.append(record)
        (OUT/'results.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    print('COMPLETE',len(summary),flush=True)
if __name__=='__main__':main()
