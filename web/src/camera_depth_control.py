"""Depth requests shared by the HTTP server and the single camera owner."""
import fcntl
import json
import os
from pathlib import Path
import time
import uuid

ROOT=Path(__file__).resolve().parents[2]
KEYS=('d435','d405','astra')


class DepthControl:
    def __init__(self,path=None,frames=None):
        self.path=Path(path) if path else ROOT/'web/hardware/runtime/camera-depth-control.json'
        self.frames=Path(frames) if frames else ROOT/'web/public/camera-live'

    def _lock(self):
        self.path.parent.mkdir(parents=True,exist_ok=True)
        lock=self.path.with_suffix('.lock').open('a+')
        fcntl.flock(lock,fcntl.LOCK_EX)
        return lock

    def _read(self):
        try:return json.loads(self.path.read_text())
        except (OSError,ValueError):return {}

    def _write(self,data):
        temp=self.path.with_suffix('.tmp');temp.write_text(json.dumps(data));os.replace(temp,self.path)

    def reset(self):
        with self._lock():
            data=dict(session=uuid.uuid4().hex,pid=os.getpid(),requested=dict.fromkeys(KEYS,False))
            self._write(data)
            return data['session']

    def enabled(self,key):
        return self._read().get('requested',{}).get(key) is True

    def snapshot(self):
        settings=self._read();result={}
        for key in KEYS:
            requested=settings.get('requested',{}).get(key) is True
            try:meta=json.loads((self.frames/(key+'.json')).read_text())
            except (OSError,ValueError):meta={}
            depth=meta.get('depth') or {}
            age=time.time()-meta.get('updated_at',0)
            fresh=meta.get('capture_session')==settings.get('session') and 0<=age<4
            acknowledged=fresh and depth.get('requested') is requested
            state=depth.get('state') if acknowledged else ('starting' if requested else 'stopping')
            if not fresh:state='unavailable'
            result[key]=dict(requested=requested,state=state,
                active=fresh and depth.get('state')=='online',
                color_online=fresh and meta.get('state')=='online',
                message=depth.get('message') or meta.get('message') or '')
        return dict(ok=True,cameras=result)

    def request(self,data):
        if not isinstance(data,dict) or set(data)!={'camera','enabled'}:
            raise ValueError('Expected camera and enabled only')
        key=data['camera'];enabled=data['enabled']
        if key not in KEYS or type(enabled) is not bool:
            raise ValueError('Unknown camera or non-boolean enabled')
        with self._lock():
            settings=self._read()
            if type(settings.get('pid')) is not int:raise RuntimeError('Camera service is not running')
            try:os.kill(settings['pid'],0)
            except OSError:raise RuntimeError('Camera service is not running')
            if enabled and not self.snapshot()['cameras'][key]['color_online']:
                raise ValueError('Color camera is offline; depth was not started')
            settings['requested'][key]=enabled;self._write(settings)
        return self.snapshot()
