"""Headless map UI check. Every request is intercepted; no robot connection."""
import argparse
import json
import mimetypes
from pathlib import Path
from playwright.sync_api import sync_playwright


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    public=Path(__file__).resolve().parents[1]/'public'
    tasks=[dict(id=key,label=label,status='pending',connected=key=='pick') for key,label in (
        ('navigate_pick','导航到取物工作区并停稳'),('pick','抓取并抽出指定电池'),
        ('carry','进入运输姿态'),('navigate_place','携物通过障碍区并到柜前停稳'),
        ('place','红框 A 口插入、释放并稳定 3 秒'))]
    task=dict(status='idle',tasks=tasks,can_start=True,can_continue=False,
              control_owned=False,command_epoch=12,logs=[],phase='navigate_pick')
    sent=[];errors=[]
    def route(request):
        url=request.request.url
        suffix=url.removeprefix('http://mission.test/')
        if suffix=='api/navigation-view':
            body=dict(competition=task,map=dict(width=20,height=20,resolution=.25,
                origin=[0,0,0],frame='map',cells=[0]*400),
                pose=dict(valid=True,x=2,y=2,yaw=0,age_s=.1),path=None)
            return request.fulfill(json=body)
        if suffix=='api/competition-task':
            data=request.request.post_data_json;sent.append(data)
            assert data['action']=='start' and data['environment_clear'] is True
            assert data['expected_epoch']==12
            task.update(status='needs_attention',reason='导航到取物工作区并停稳：能力未接入',
                        can_continue=True,run_id='browser-fixture')
            return request.fulfill(status=202,json=task)
        target=(public/suffix).resolve()
        if target.is_relative_to(public.resolve()) and target.is_file():
            return request.fulfill(body=target.read_bytes(),content_type=mimetypes.guess_type(target)[0] or 'text/plain')
        return request.abort()
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='msedge',headless=True)
        page=browser.new_page(viewport=dict(width=1360,height=900))
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.route('**/*',route)
        page.goto('http://mission.test/map.html')
        page.get_by_text('等待比赛',exact=True).wait_for()
        assert page.locator('#competition-start').is_disabled()
        page.locator('#environment-clear').check()
        page.locator('#competition-start').click()
        page.get_by_text('导航到取物工作区并停稳：能力未接入',exact=True).wait_for()
        assert page.locator('#competition-continue').is_enabled()
        assert page.locator('#competition-stop').is_disabled()
        assert len(sent)==1 and not errors,(sent,errors)
        page.screenshot(path=str(args.output/'competition-desktop.png'),full_page=True)
        page.set_viewport_size(dict(width=390,height=844))
        page.screenshot(path=str(args.output/'competition-mobile.png'),full_page=True)
        assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
        browser.close()
    print(json.dumps(dict(browser_errors=errors,mocked_start_requests=len(sent),desktop=True,mobile=True)))


if __name__=='__main__':main()
