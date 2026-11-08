"""Read-only browser check. Isolates the viewer; never loads control scripts."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import time
import urllib.request
from email.utils import formatdate
from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://192.168.58.14:9983")
    parser.add_argument("--screenshot", type=Path)
    args = parser.parse_args()
    base = args.base_url.rstrip("/")
    with urllib.request.urlopen(base + "/", timeout=10) as response:
        source = response.read().decode()
        policy = response.headers.get("Content-Security-Policy", "")
    isolated = re.sub(r"<script\b[^>]*>.*?</script>", lambda match: match.group() if 'src="js/robot-preview.js"' in match.group() else "", source, flags=re.S)
    requests, errors = [], []
    fixture = {"mode": "live"}
    edge = Path("C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe")
    executable = str(edge) if edge.exists() else shutil.which("google-chrome") or shutil.which("chromium")
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path=executable, headless=True,
            args=["--no-first-run", "--disable-background-networking", "--enable-unsafe-swiftshader"])
        page = browser.new_page(viewport={"width": 1520, "height": 1060})
        page.on("request", lambda request: requests.append((request.method, request.url)))
        page.on("pageerror", lambda error: errors.append(str(error)))
        # Proxy fixture assets through the test client: a fulfilled root has no network address space in Edge.
        page.route(base + "/**", lambda route: route.fulfill(response=route.fetch()))
        page.route(base + "/", lambda route: route.fulfill(status=200, content_type="text/html; charset=utf-8",
            headers={"Content-Security-Policy": policy}, body=isolated))

        def state_route(route):
            if fixture["mode"] == "live":
                return route.fulfill(response=route.fetch())
            now = time.time()
            data = {"time": now, "ros": {"connected": True}, "arms": {
                side: {"fresh": True, "timestamp": now, "age_s": 0, "joints": [(.2 if side == "left" else -.4)] * 6}
                for side in ("left", "right")},
                "body": {"fresh": True, "timestamp": now, "age_s": 0, "height": 8.308}}
            if fixture["mode"] == "frozen":
                if "frozen_data" not in fixture:
                    fixture["frozen_data"] = data
                data = fixture["frozen_data"]
            if fixture["mode"] == "stale":
                for item in [*data["arms"].values(), data["body"]]:
                    item.update(fresh=False, timestamp=now - 20, age_s=20)
            route.fulfill(status=200, content_type="application/json", headers={"Date": formatdate(now, usegmt=True)}, body=json.dumps(data))
        page.route(base + "/api/state", state_route)
        page.goto(base + "/", wait_until="domcontentloaded", timeout=25000)
        page.locator('#robot-preview[data-loaded="true"], #preview-model-status.preview-error').first.wait_for(timeout=45000)
        assert page.locator("#robot-preview").get_attribute("data-loaded") == "true", page.locator("#preview-model-status").inner_text()
        page.locator("#preview-right-status.online").wait_for(timeout=10000)
        assert page.locator("#robot-preview-viewport canvas").count() == 1
        assert not errors, errors
        print("PASS: actual WebGL/URDF/21-STL load and live feedback")
        if args.screenshot:
            args.screenshot.parent.mkdir(parents=True, exist_ok=True)
            page.screenshot(path=str(args.screenshot))
        fixture["mode"] = "healthy"
        page.locator("#preview-right-joints").filter(has_text="-22.9").wait_for(timeout=8000)
        assert "11.5" in page.locator("#preview-left-joints").inner_text()
        assert "200.0 mm" in page.locator("#preview-lift-value").inner_text()
        print("PASS: independent left/right/lift fixture display")
        fixture["mode"] = "frozen"
        page.wait_for_timeout(3200)
        assert "online" not in (page.locator("#preview-right-status").get_attribute("class") or ""), "Repeated frozen response remains live"
        print("PASS: repeated frozen whole responses expire against HTTP Date")
        fixture["mode"] = "stale"
        page.locator("#preview-right-status:not(.online)").wait_for(timeout=8000)
        assert page.locator("#preview-right-joints").inner_text() == "—"
        assert page.locator("#preview-lift-value").inner_text() == "—"
        print("PASS: expired feedback is unavailable")
        page.set_viewport_size({"width": 480, "height": 900})
        page.wait_for_timeout(300)
        assert page.locator("#robot-preview").bounding_box()["width"] <= 480, "Preview overflows viewport"
        assert page.locator("#robot-preview-viewport canvas").bounding_box()["width"] > 200
        print("PASS: mobile viewport")
        assert all(method == "GET" for method, _ in requests), requests
        assert not any("/api/command" in url or "/events" in url or "/api/camera/" in url for _, url in requests)
        assert not errors, errors
        print("PASS: viewer performs GET only; no command, SSE or camera control")
        print("Browser requests:", len(requests))
        browser.close()


if __name__ == "__main__":
    main()
