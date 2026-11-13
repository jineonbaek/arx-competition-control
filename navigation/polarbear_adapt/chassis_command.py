"""Local ARX control console transport and low-speed planar command."""
import json
import math
from http.cookiejar import CookieJar
from urllib.request import build_opener, HTTPCookieProcessor, ProxyHandler, Request


class WebChassis:
    sdk_units = False
    native_sdk = False
    origin = 'http://127.0.0.1:9983'

    def __init__(self):
        self.client = build_opener(ProxyHandler({}), HTTPCookieProcessor(CookieJar()))
        with self.client.open(self.origin + '/', timeout=.2):
            pass
        self.sequence = 0

    def send(self, velocity):
        x, y, yaw = map(float, velocity)
        if self.sdk_units and not self.native_sdk:
            # Existing console: input_scale=1, hardware divides by 2.5; XY cap=2.
            scale=min(2.5, 1.999999 / max(math.hypot(x,y),1e-9))
            x,y,yaw=x*scale,y*scale,yaw*2.5
        self.sequence += 1
        payload = dict(type='chassis.nav_sdk' if self.sdk_units and self.native_sdk else 'chassis', x=x, y=y, omega=yaw) if any(velocity) else dict(type='chassis.stop')
        payload['sequence'] = self.sequence
        request = Request(self.origin + '/api/command', json.dumps(payload).encode(),
                          {'Origin': self.origin, 'Content-Type': 'application/json'})
        with self.client.open(request, timeout=.18) as reply:
            if not json.load(reply).get('ok'):
                raise RuntimeError('控制台拒绝了底盘指令')


def follow_velocity(forward, left, yaw_error):
    gain = min(.8, .08 / max(math.hypot(forward, left), 1e-6))
    return forward * gain, left * gain, max(-.18, min(.18, yaw_error))
