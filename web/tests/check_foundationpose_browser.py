"""Read-only browser check; all capture/inference POSTs are intercepted."""
import json
from pathlib import Path
import shutil
import sys
from playwright.sync_api import sync_playwright

result=json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
base='http://192.168.58.14:9983'
with sync_playwright() as pw:
    edge=Path('C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe')
    exe=str(edge) if edge.exists() else shutil.which('chromium') or shutil.which('google-chrome')
    browser=pw.chromium.launch(headless=True,executable_path=exe)
    context=browser.new_context(viewport={'width':1440,'height':1100})
    state=dict(status='idle',running=False,message='idle')
    view=dict(id=result['id'],width=640,height=480,time=1794400000,image=result['mask_image'],paired_ms=10)
    actions=[];errors=[];real_posts=[]
    def guard(route):
        req=route.request
        if req.url.split('?')[0].endswith('/api/foundationpose'):
            if req.method=='POST':
                data=req.post_data_json;actions.append(data['op'])
                if data['op']=='capture':
                    state.update(status='captured',running=False,message='captured',view=view,
                                 depth_image=result['mask_image'],valid_depth_fraction=.8)
                elif data['op']=='infer':
                    assert data['id']==result['id'] and min(data['roi'][2]-data['roi'][0],data['roi'][3]-data['roi'][1])>=12
                    state.update(status='running',running=True,run_id='demo',message='GPU computing',result=None)
                elif data['op']=='stop':
                    state.update(status='stopped',running=False,message='released')
                else:raise AssertionError(data)
            elif state.get('running') and actions.count('infer')==1:
                state.update(status='complete',running=False,result=result,message='released')
            route.fulfill(status=200,content_type='application/json',body=json.dumps(state))
        elif req.method=='GET' and not req.url.endswith('/events'):
            route.continue_()
        else:
            if req.method=='POST':real_posts.append(req.url)
            route.abort()
    context.route('**/*',guard)
    page=context.new_page();page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto(base,wait_until='domcontentloaded')
    assert page.locator('[data-camera=d405] a[href="foundationpose.html"]').count()==1
    page.goto(base+'/foundationpose.html',wait_until='domcontentloaded')
    assert page.locator('input[type=checkbox]').count()==0
    assert page.locator('#estimate').is_disabled()
    page.click('#capture')
    page.locator('#depth').wait_for(state='visible')
    box=page.locator('#photo').bounding_box()
    page.mouse.move(box['x']+box['width']*.1,box['y']+box['height']*.4)
    page.mouse.down();page.mouse.move(box['x']+box['width']*.4,box['y']+box['height']*.8);page.mouse.up()
    assert page.locator('#estimate').is_enabled()
    page.click('#estimate')
    page.locator('#overlay').wait_for(state='visible')
    assert 'GiB' in page.inner_text('#metrics')
    assert '26 mm' in page.inner_text('#grip-geometry')
    assert page.locator('#grip-diagram').evaluate('(e)=>e.naturalWidth')>0
    if result.get('grasp_reference'):
        assert 'G 夹持中心 XYZ' in page.inner_text('#metrics')
        assert '−90' in page.inner_text('#grip-pose-status')
    assert page.locator('#cancel').is_disabled()
    page.click('#estimate')
    page.locator('#cancel').wait_for(state='visible')
    page.click('#cancel')
    assert not state['running']
    assert actions==['capture','infer','infer','stop'],actions
    assert not real_posts,real_posts
    assert not errors,errors
    page.screenshot(path=str(Path(sys.argv[1]).with_suffix('.png')),full_page=True)
    print('PASS: D405 entry, no model switch, RGB-D fixture, ROI, real worker-result display, automatic release and cancel. No hardware POST.')
    browser.close()
