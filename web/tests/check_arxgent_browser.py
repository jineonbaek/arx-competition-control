"""Offline browser check. Requires development-only Playwright; no cloud or robot calls."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from http.server import ThreadingHTTPServer

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'web/src'), str(ROOT / 'web/tests')]
from server import make_handler
from control import Controller
from test_control import FakeAdapter, configured
from arxgent import ArxgentChat
from agent.config import Config
from agent.runtime import AgentRuntime
from playwright.sync_api import expect, sync_playwright


class Model:
    def __init__(self, store):
        self.store = store

    def complete(self, messages, tools):
        assert {d['function']['name'] for d in tools} == {'get_system_status', 'observe_scene', 'read_observation'}
        text = messages[-1]['content']
        if '慢一点' in text:
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline and not self.store.cancelled:
                time.sleep(.03)
        answer = '<img src=x onerror=alert(1)>' if '<img' in text else ('你刚才说的是红色电池。' if '什么颜色' in text else '已记录：' + text)
        return {'role': 'assistant', 'content': answer}


def runtime(config, store, tools, **kwargs):
    return AgentRuntime(config, store, tools, model=Model(store), **kwargs)


def main():
    with tempfile.TemporaryDirectory() as temporary:
        chat = ArxgentChat(Config(profiles={'cloud': {'model': 'fixture', 'base_url': 'http://127.0.0.1:1', 'stream': False}}), Path(temporary), runtime_factory=runtime)
        config = configured()
        config['execution_enabled'] = True
        adapter = FakeAdapter()
        controller = Controller(config, adapter)
        handler = make_handler(controller, chat)
        handler.log_message = lambda *args: None
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
        server.daemon_threads = True
        server.stopping = threading.Event()
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

        def feedback():
            while not server.stopping.is_set():
                controller.update_body(0., [0.] * 4, 0., 0., 0.)
                for arm in ('left', 'right'):
                    controller.update_arm(arm, [0.] * 6, -1., [0.] * 6)
                server.stopping.wait(.05)

        pulse = threading.Thread(target=feedback, daemon=True)
        pulse.start()
        try:
            with sync_playwright() as playwright:
                browser = playwright.chromium.launch(headless=True)
                page = browser.new_page(viewport={'width': 1440, 'height': 1100})
                errors, commands = [], []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.route('**/camera-live/**', lambda route: route.fulfill(status=200, content_type='application/json', body='{"state":"offline"}'))

                def command(route):
                    commands.append(route.request.post_data_json)
                    route.fulfill(status=200, content_type='application/json', body='{"ok":true,"message":"test interception"}')

                page.route('**/api/command', command)
                page.goto(f'http://127.0.0.1:{server.server_port}/', wait_until='domcontentloaded')
                page.locator('#arxgent-status').filter(has_text='已连接').wait_for()
                agent = page.locator('#arxgent-panel').bounding_box()
                arm = page.locator('.arm-panel').bounding_box()
                chassis = page.locator('[aria-labelledby="chassis-title"]').bounding_box()
                camera = page.locator('#camera-panel').bounding_box()
                lift = page.locator('[aria-labelledby="lift-title"]').bounding_box()
                assert arm['x'] + arm['width'] <= camera['x']
                assert camera['x'] == agent['x'] and camera['y'] + camera['height'] <= agent['y']
                assert agent['x'] + agent['width'] <= chassis['x']
                assert chassis['y'] + chassis['height'] <= lift['y']
                textbox = page.locator('#arxgent-input')
                textbox.fill('请记住红色电池')
                textbox.press('Enter')
                page.locator('.arxgent-message:not(.user)').filter(has_text='已记录').wait_for()
                expect(page.locator('.arxgent-message:not(.user) .arxgent-message-label')).to_contain_text('思考耗时（含工具）')
                expect(page.locator('.arxgent-message:not(.user) .arxgent-message-label')).to_contain_text('工具：无')
                assert page.locator('.arxgent-message-content').first.evaluate('(node) => parseFloat(getComputedStyle(node).lineHeight)') <= 18
                assert page.locator('.arxgent-message').first.evaluate('(node) => parseFloat(getComputedStyle(node).paddingTop)') <= 7
                textbox.fill('我刚才说的是什么颜色')
                page.locator('#arxgent-send').click()
                page.locator('.arxgent-message:not(.user)').filter(has_text='你刚才说的是红色电池').wait_for()
                assert page.locator('.arxgent-message').count() == 4
                page.reload(wait_until='domcontentloaded')
                page.locator('.arxgent-message').nth(3).wait_for()
                toggle = page.locator('#chassis-motion-toggle')
                toggle.click()
                assert toggle.get_attribute('aria-pressed') == 'true'
                textbox = page.locator('#arxgent-input')
                textbox.fill('w a s d q e space')
                assert toggle.get_attribute('aria-pressed') == 'false'
                assert not any(c.get('type') == 'chassis' for c in commands)
                page.locator('#arxgent-new').click()
                expect(page.locator('.arxgent-message')).to_have_count(0)
                textbox.fill('<img src=x onerror=alert(1)>')
                textbox.press('Enter')
                page.locator('.arxgent-message:not(.user)').wait_for()
                assert page.locator('#arxgent-messages img').count() == 0
                assert '<img' in page.locator('.arxgent-message:not(.user)').inner_text()
                textbox.fill('请慢一点回答')
                textbox.press('Enter')
                page.locator('#arxgent-cancel').wait_for(state='visible')
                page.locator('#arxgent-cancel').click()
                expect(page.locator('#arxgent-activity')).to_contain_text('已取消')
                page.set_viewport_size({'width': 390, 'height': 844})
                page.locator('#arxgent-panel').scroll_into_view_if_needed()
                assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
                assert not errors, errors
                assert not adapter.calls, adapter.calls
                print(json.dumps({'desktop_three_columns': True, 'camera_above_agent': True, 'chassis_above_lift': True, 'two_turns_and_reload': True,
                                  'input_exits_motion_mode': True, 'motion_requests': 0, 'html_rendered_as_text': True,
                                  'cancel_completed': True, 'mobile_no_horizontal_overflow': True, 'page_errors': errors}))
                browser.close()
        finally:
            chat.close()
            server.stopping.set()
            server.shutdown()
            server.server_close()
            thread.join()
            pulse.join()


if __name__ == '__main__':
    main()
