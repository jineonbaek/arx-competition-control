"""Checks the D405 entry, real saved candidates and read-only browser interactions."""
import shutil
from pathlib import Path
from playwright.sync_api import sync_playwright

BASE='http://192.168.58.14:9983'
with sync_playwright() as pw:
    edge=Path('C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe')
    exe=str(edge) if edge.exists() else shutil.which('chromium') or shutil.which('google-chrome')
    browser=pw.chromium.launch(headless=True,executable_path=exe,args=['--enable-unsafe-swiftshader'])
    page=browser.new_page(viewport={'width':1440,'height':1000})
    errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
    def read_only(route):
        if route.request.method=='GET' and not route.request.url.endswith('/events'):route.continue_()
        else:route.abort()
    page.route('**/*',read_only)
    page.goto(BASE,wait_until='domcontentloaded')
    link=page.locator('[data-camera=d405] a[href="grasp.html"]')
    assert link.get_attribute('target')=='_blank'
    with page.expect_popup() as popup:link.click()
    view=popup.value
    view.on('pageerror',lambda e:errors.append(str(e)))
    view.locator('#candidate option').first.wait_for(state='attached',timeout=30000)
    n=view.locator('#candidate option').count()
    assert n>=2
    assert view.locator('#viewport canvas').count()==1
    view.click('#next')
    assert view.input_value('#candidate')=='1'
    assert 'body4' in view.inner_text('#metrics')
    print('rendered',n,'candidates;',view.inner_text('#metrics'))
    b=view.locator('#photo').bounding_box()
    view.mouse.move(b['x']+b['width']*.3,b['y']+b['height']*.3)
    view.mouse.down();view.mouse.move(b['x']+b['width']*.6,b['y']+b['height']*.6,steps=5);view.mouse.up()
    assert view.locator('#infer').is_enabled() and view.locator('#candidate').is_disabled()
    assert not errors,errors
    browser.close()
