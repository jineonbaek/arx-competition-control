"""Exercise the actual handeye page with intercepted motion actions (never forwarded)."""
import argparse
import json
import re
import shutil
import time
from pathlib import Path
from urllib.parse import urlsplit
from playwright.sync_api import sync_playwright


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--base-url",default="http://192.168.58.14:9983")
    parser.add_argument("--screenshot",type=Path)
    args=parser.parse_args()
    base=args.base_url.rstrip("/")
    edge=Path("C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe")
    exe=str(edge) if edge.exists() else shutil.which("google-chrome") or shutil.which("chromium")
    state=dict(time=0,session=None,pending=None,active=False,history=[],board_configured=False,cursor=0,total=13)
    ops=[];errors=[]
    def route_request(route):
        path=urlsplit(route.request.url).path
        if path=="/api/handeye":
            if route.request.method=="POST":
                op=route.request.post_data_json["op"];ops.append(op)
                if op=="begin":
                    assert route.request.post_data_json["marker"]==26
                    state.update(session="fixture",board_configured=True,cursor=1,pending=dict(
                        point=1,wrist_rise_mm=34.6,start=[0]*6,target=[0,0,.087,.052,0,0],ticket="fake",expires=time.time()+60,
                        check=dict(clear=True,lower_mm=115,margin_mm=20,link="right_link23")),
                        history=[dict(point=0,status="跳过",message="候选间隙不足")])
                elif op=="execute":
                    assert route.request.post_data_json["ticket"]=="fake"
                    state.update(pending=None,history=[dict(point=1,status="不可用",message="目标标记不可见")])
                else:raise AssertionError(op)
            route.fulfill(status=200,content_type="application/json",body=json.dumps(dict(state,time=time.time())))
        elif route.request.method=="GET" and path!="/events":
            route.fulfill(response=route.fetch())
        else:
            route.abort()
            errors.append("unexpected request: "+path)
    with sync_playwright() as pw:
        browser=pw.chromium.launch(headless=True,executable_path=exe,args=["--enable-unsafe-swiftshader","--no-first-run"])
        page=browser.new_page(viewport=dict(width=1440,height=1050))
        page.on("pageerror",lambda e:errors.append(str(e)))
        page.route("**/*",route_request)
        page.goto(base+"/handeye.html",wait_until="domcontentloaded")
        page.locator('#robot-preview[data-loaded=true]').wait_for(timeout=60000)
        assert page.locator('#handeye-start').is_disabled()
        assert ops == [], 'Opening the page must not start a calibration operation'
        assert [page.input_value('#'+key) for key in ('board-dict','board-id','board-mm')]==['5X5_100','26','11.25']
        page.click('#step-begin')
        page.locator('#step-detail').filter(has_text='115 mm').wait_for()
        assert '34.6 mm' in page.inner_text('#step-detail')
        assert ops==["begin"] and page.locator('#handeye-start').is_disabled()
        page.check('#step-confirm');page.click('#handeye-start')
        page.locator('#step-history').filter(has_text='不可用').wait_for()
        assert ops==["begin","execute"] and page.locator('#handeye-start').is_disabled()
        assert not errors,errors
        print("PASS: real WebGL + right-arm preview, skip/status, explicit click, sample rejection; all movement actions intercepted")
        browser.close()

if __name__=="__main__":main()
