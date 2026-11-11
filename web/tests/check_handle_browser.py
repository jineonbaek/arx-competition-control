"""Browser regression for retained handle preview. POSTs are mocked, never sent to hardware."""
import json
import shutil
from pathlib import Path
from playwright.sync_api import sync_playwright

BASE = 'http://192.168.58.14:9983'
with sync_playwright() as pw:
    edge=Path('C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe')
    exe=str(edge) if edge.exists() else shutil.which('chromium') or shutil.which('google-chrome')
    browser=pw.chromium.launch(headless=True,executable_path=exe,args=['--enable-unsafe-swiftshader'])
    context=browser.new_context(viewport={'width':1440,'height':1000})
    errors=[]; posts=[]
    def guarded(route):
        req=route.request
        if req.method=='GET' and not req.url.endswith('/events'):
            route.continue_()
        elif req.method=='POST' and req.url.endswith('/api/handle-preview'):
            data=req.post_data_json
            posts.append(data)
            assert data['op']=='plan'
            roi=data['roi']
            result=dict(id=data['id'],roi=roi,pre_mm=[350,-250,700],feature_mm=[400,-250,700],
                        pixels=[[200,180],[220,180]],inliers=100,valid=110,
                        wrist_pre_mm=[230,-250,700],wrist_end_mm=[280,-250,700],
                        teach=dict(offset_mm=[-120,0,0],wrist_pose=[280,-250,700,-90,0,0]))
            route.fulfill(status=200,content_type='application/json',body=json.dumps(result))
        else:
            route.abort()
    context.route('**/*',guarded)
    page=context.new_page()
    page.on('pageerror',lambda error:errors.append(str(error)))
    page.goto(BASE,wait_until='domcontentloaded')
    assert page.locator('a[href="grasp.html"]').count()==0
    link=page.locator('[data-camera=d405] a[href="handle.html"]')
    assert link.get_attribute('target')=='_blank'
    page.goto(BASE+'/handle.html',wait_until='domcontentloaded')
    page.wait_for_function("() => document.getElementById('status').textContent.includes('已加载')")
    assert page.locator('#viewport canvas').count()==1
    box=page.locator('#photo').bounding_box()
    page.mouse.move(box['x']+box['width']*.3,box['y']+box['height']*.3)
    page.mouse.down()
    page.mouse.move(box['x']+box['width']*.6,box['y']+box['height']*.6,steps=5)
    page.mouse.up()
    assert page.locator('#plan').is_enabled()
    page.click('#plan')
    page.wait_for_function("() => document.getElementById('status').textContent.includes('预览完成')")
    assert '280.0 / -250.0 / 700.0' in page.inner_text('#metrics')
    assert page.locator('#teach').is_disabled()
    page.check('#confirm')
    assert page.locator('#teach').is_enabled()
    page.uncheck('#confirm')
    page.click('#reset')
    assert len(posts)==1
    assert not errors,errors
    print('PASS: entry, saved RGB-D/cloud, ROI, mocked segment/offset display, teach confirmation, orbit reset. No real POST.')
    browser.close()
