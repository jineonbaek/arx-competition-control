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

    def _expire_locked(self,settings):
        scope=settings.get('handeye')
        if scope and scope['expires'] <= time.monotonic():
            self._release_locked(settings,scope)

    def _release_locked(self,settings,scope):
        # If the worker fell back from a failed depth stream, preserve its fallback.
        camera=scope.get('camera','d405')
        if settings['requested'][camera] == scope['enabled']:
            settings['requested'][camera]=scope['previous']
        settings.pop('handeye',None)
        self._write(settings)

    def acquire_handeye(self, enabled):
        return self._acquire(enabled,'handeye','d405')

    def acquire_grasp(self, camera='d405'):
        return self._acquire(True,'grasp',camera)

    def _acquire(self, enabled, owner, camera):
        if camera not in ('d435','d405'):raise ValueError('Unsupported RGB-D camera')
        if type(enabled) is not bool:raise ValueError('Invalid calibration depth mode')
        with self._lock():
            settings=self._read()
            self._expire_locked(settings)
            active=settings.get('handeye')
            if active:
                if owner=='handeye' and active.get('owner')=='grasp':
                    self._release_locked(settings,active)
                else:raise RuntimeError('已有相机正在执行另一项临时采样')
            if type(settings.get('pid')) is not int:raise RuntimeError('相机采集服务未运行')
            os.kill(settings['pid'],0)
            token=uuid.uuid4().hex
            settings['handeye']=dict(token=token,owner=owner,camera=camera,enabled=enabled,previous=settings['requested'][camera],
                                     expires=time.monotonic()+45)
            settings['requested'][camera]=enabled
            self._write(settings)
            return token

    def release_handeye(self,token):
        with self._lock():
            settings=self._read()
            scope=settings.get('handeye')
            if scope and scope['token']==token:self._release_locked(settings,scope)

    def _current(self):
        settings=self._read()
        if settings.get('handeye',{}).get('expires',float('inf'))<=time.monotonic():
            with self._lock():
                settings=self._read()
                self._expire_locked(settings)
        return settings

    def enabled(self,key):
        return self._current().get('requested',{}).get(key) is True

    def snapshot(self,settings=None):
        if settings is None:settings=self._current()
        result={}
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
                handeye_priority=bool(settings.get('handeye')) and key==settings['handeye'].get('camera','d405'),
                active=fresh and depth.get('state')=='online',
                color_online=fresh and meta.get('state')=='online',
                message=depth.get('message') or meta.get('message') or '')
        return dict(ok=True,cameras=result)

    def manual_request(self,data):
        return self.request(data,manual=True)

    def request(self,data,*,manual=False):
        if not isinstance(data,dict) or set(data)!={'camera','enabled'}:
            raise ValueError('仅接受 camera 和 enabled 参数')
        key=data['camera'];enabled=data['enabled']
        if key not in KEYS or type(enabled) is not bool:
            raise ValueError('相机名称或深度开关参数无效')
        with self._lock():
            settings=self._read()
            self._expire_locked(settings)
            if manual and settings.get('handeye') and key==settings['handeye'].get('camera','d405'):
                raise RuntimeError('深度由正在采样的程序管理')
            if type(settings.get('pid')) is not int:raise RuntimeError('相机采集服务未运行')
            try:os.kill(settings['pid'],0)
            except OSError:raise RuntimeError('相机采集服务未运行')
            if enabled and not self.snapshot(settings)['cameras'][key]['color_online']:
                raise ValueError('彩色相机未就绪，未开启深度')
            settings['requested'][key]=enabled;self._write(settings)
        return self.snapshot()
