"""Optional, process-local advice. The robot thread never contacts a model or a network peer."""
from collections import deque
import base64
from contextlib import contextmanager
from contextvars import ContextVar
import copy
import io
import itertools
import json
import logging
import math
import secrets
import threading
import time
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
CURRENT = ContextVar('arx_advice_session', default=None)
ROUTE_CHOICES = ('elbow_first', 'shoulder_first', 'wrist_first')
TTL = 120.


def box(value):
    return (isinstance(value, list) and len(value) == 4 and
            all(type(v) is int and 0 <= v <= 1000 for v in value) and
            value[2] - value[0] >= 12 and value[3] - value[1] >= 12)


def valid(kind, answer):
    if not isinstance(answer,dict) or set(answer)!={'action','battery','rim','seed_deg','reason'}:
        return False
    if not isinstance(answer['reason'],str) or len(answer['reason'])>300:
        return False
    a,b,r,s=(answer[k] for k in ('action','battery','rim','seed_deg'))
    if a=='abstain':return b==r==s==[]
    if kind=='route':
        return a in ROUTE_CHOICES and b==r==s==[]
    if kind=='ik':
        return a=='seed' and b==r==[] and isinstance(s,list) and len(s)==6 and all(
            type(v) in (int,float) and math.isfinite(v) and abs(v)<=25 for v in s)
    return (kind in ('grasp','edges') and a=='region' and s==[] and box(b) and
            (r==[] if kind=='grasp' else box(r)))


def pixels(bounds,size,padding=0):
    w,h=size
    x,y,u,v=bounds
    return [max(0, w*x//1000-padding),max(0,h*y//1000-padding),
            min(w,(w*u+999)//1000+padding),min(h,(h*v+999)//1000+padding)]


def photo(source):
    if isinstance(source,(str,Path)):
        with Image.open(source) as im:return photo(im)
    im=source.copy() if isinstance(source,Image.Image) else Image.fromarray(source)
    im.thumbnail((720,540))
    buf=io.BytesIO();im.convert('RGB').save(buf,'JPEG',quality=79)
    return base64.b64encode(buf.getvalue()).decode('ascii')


def photos(image,kind):
    images=[photo(image)]
    try:
        if kind=='edges':
            ref=json.loads((ROOT/'config/seated-reference.json').read_text())['reference_id']
            if Path(ref).name==ref:images.append(photo(ROOT/'reports'/ref/'d405.jpg'))
        else:
            ref=Path('/home/arx/arx-vision-demo/vision/config/battery-reference.png')
            if ref.is_file():images.append(photo(ref))
    except (OSError,ValueError,KeyError):pass
    return images


class Mailbox:
    def __init__(self,clock=time.monotonic):
        self.clock=clock;self.lock=threading.Lock();self.jobs={}
        self.enabled=False
        self.events=deque(maxlen=200);self.worker_seen=None;self.sequence=0

    def event(self,event,job=None,**details):
        # Called under the mailbox lock; no network, no image payload in history.
        self.sequence+=1
        row=dict(seq=self.sequence,time=time.time(),event=event,**details)
        if job:row.update(id=job['id'],run=job['run'],kind=job['kind'])
        self.events.append(row)

    def _prune(self):
        for key,job in list(self.jobs.items()):
            if self.clock()-job['created']>=TTL:
                self.event('expired',job);self.jobs.pop(key,None)
        while len(self.jobs)>32:
            key=next(iter(self.jobs));self.event('superseded',self.jobs.pop(key))

    def snapshot(self):
        with self.lock:
            self._prune()
            return dict(enabled=self.enabled,worker_online=self.worker_seen is not None and self.clock()-self.worker_seen<8.,
                pending=sum(j['answer'] is None for j in self.jobs.values()),events=list(self.events))

    def heartbeat(self):
        with self.lock:self.worker_seen=self.clock()

    def diagnostic(self,run,stage,reason,evidence=None):
        if not self.enabled:return None
        reason=str(reason)[:500]
        with self.lock:
            self._prune()
            for j in self.jobs.values():
                if j['run']==run and j['kind']=='diagnostic' and j['evidence'].get('stage')==stage and j['evidence'].get('reason')==reason:return j['id']
        detail=dict(stage=stage,reason=reason,details=evidence or {})
        # Read existing fresh files only. A diagnosis never waits for a camera,
        # enables depth, moves a viewpoint, or treats a photo as a motion receipt.
        if any(word in stage for word in ('D405','放置','推入','抓取')):
            camera='d435' if '抓取' in stage else 'd405'
            folder=ROOT/'web/public/camera-live'
            try:
                meta=json.loads((folder/(camera+'.json')).read_text())
                name=meta['raw_image_file'];stamp=float(meta['captured_at'])
                if Path(name).name==name and 0<=time.time()-stamp<4.:
                    detail.update(images=[photo(folder/name)],camera=camera,image_time=stamp)
            except (OSError,ValueError,KeyError,TypeError):pass
        return self.open(run,'diagnostic',detail)

    def applied(self,key):
        with self.lock:
            job=self.jobs.get(key)
            if job and job['answer']:self.event('applied',job,reason=job['answer']['reason'])

    def open(self,run,kind,evidence):
        key=secrets.token_hex(16)
        with self.lock:
            self._prune()
            self.jobs[key]=dict(id=key,run=run,kind=kind,evidence=evidence,created=self.clock(),
                                leased=False,answer=None)
            self._prune()
            self.event('requested',self.jobs[key],stage=evidence.get('stage',''),reason=str(evidence.get('reason',''))[:500])
        return key

    def drop(self,key):
        with self.lock:
            job=self.jobs.pop(key,None)
            if job:self.event('closed',job)

    def expired(self,key):
        with self.lock:
            job=self.jobs.get(key)
            return job is None or self.clock()-job['created']>=TTL

    def take(self):
        with self.lock:
            self.worker_seen=self.clock();self._prune()
            for job in self.jobs.values():
                if not job['leased'] and self.clock()-job['created']<TTL:
                    job['leased']=True
                    self.event('received',job)
                    return {k:job[k] for k in ('id','run','kind','evidence')}
        return None

    def reply(self,key,answer):
        with self.lock:
            job=self.jobs.get(key)
            if (not job or not job['leased'] or job['answer'] is not None or
                self.clock()-job['created']>=TTL or not valid(job['kind'],answer)):return False
            job['answer']=copy.deepcopy(answer)
            self.event('replied',job,action=answer['action'],reason=answer['reason'])
            return True

    def answer(self,key):
        with self.lock:
            job=self.jobs.get(key)
            if job and self.clock()-job['created']<TTL:return job['answer']


MAIL = Mailbox()


class Episode:
    def __init__(self,run,pose,log,mail):
        self.run=run;self.pose=pose;self.log=log;self.mail=mail
        self.entries={}
        self.ik_search_spent=0.

    def applied(self,kind):
        entry=self.entries.get(kind)
        if entry and entry['id']:self.mail.applied(entry['id'])

    def end(self,kind):
        entry=self.entries.pop(kind,None)
        if entry and entry['id']:self.mail.drop(entry['id'])

    def _entry(self,kind,binding):
        pose=self.pose()
        old=self.entries.get(kind)
        if old:
            prior=old['pose']
            if (pose is None or prior[0]!=pose[0] or old['binding']!=binding or
                any(abs(x-y)>t for x,y,t in zip(prior[1],pose[1],prior[2]))):
                self.end(kind);old=None
        if old is None and pose is not None:
            old=dict(pose=pose,binding=binding,last=None,count=0,id=None,used=False)
            self.entries[kind]=old
        return old

    def failed(self,kind,binding,stamp,make_evidence,*,threshold=None):
        entry=self._entry(kind,binding)
        if not entry or entry['last']==stamp:return
        entry['last']=stamp;entry['count']+=1
        if entry['count']<(threshold if threshold is not None else (1 if kind in ('ik','route') else 3)):return
        if entry['id']:
            if not self.mail.expired(entry['id']):return
            self.mail.drop(entry['id'])
            entry.update(id=None,used=False)
        try:
            evidence=make_evidence()
            entry['id']=self.mail.open(self.run,kind,evidence)
            self.log('requested',kind)
        except (ValueError,OSError,TypeError,KeyError) as e:
            logging.getLogger(__name__).debug('Advice evidence unavailable: %s',e)

    def frame_matches(self,frame):
        pose=self.pose()
        if pose is None:return False
        actual=[*frame['joints'],float(frame['lift_rad'])]
        return len(actual)==7 and all(math.isfinite(x) and abs(x-y)<=t for x,y,t in zip(actual,pose[1],pose[2]))

    def suggestion(self,kind,binding):
        entry=self._entry(kind,binding)
        if not entry or not entry['id']:return None
        result=self.mail.answer(entry['id'])
        if result and not entry['used']:
            self.log(result['action'],kind);entry['used']=True
        return result


@contextmanager
def session(run,controller,active,log,mail=MAIL):
    if not mail.enabled:
        yield
        return
    def pose():
        if not active():return None
        snap=controller.snapshot();a=snap['arms']['right'];b=snap['body']
        if snap.get('chassis_active') or not a.get('fresh') or not b.get('fresh'):return None
        values=[*a['joints'],b['height'],b.get('waist',0.),b.get('head_yaw',0.),b.get('head_pitch',0.)]
        if len(values)!=10 or not all(math.isfinite(v) for v in values):return None
        return (snap['command_epoch'],values,[math.radians(.5)]*6+[.05,.01,.01,.01])
    ctx=Episode(run,pose,log,mail)
    token=CURRENT.set(ctx)
    try:yield
    finally:
        for key in list(ctx.entries):ctx.end(key)
        CURRENT.reset(token)


def alternate_ik(solve,q,lo,hi,goal,ctx):
    """Alternate optimizer seeds only. solve() checks FK and the sweep from the real start."""
    import numpy as np
    try:
        solution=solve(q)
        if ctx:ctx.end('ik')
        return solution
    except ValueError as error:
        if ctx is None:raise
        first=str(error)
    binding=tuple(np.round(np.r_[q,goal],5))
    def seed(offset):return solve(np.clip(q+np.radians(offset),lo,hi))
    def elbow(angle):return [0.,angle,-2.*angle,angle,0.,0.]
    for angle in (3.,-3.):
        try:solution=seed(elbow(angle))
        except ValueError:continue
        ctx.end('ik')
        return solution
    ctx.failed('ik',binding,time.monotonic_ns(),lambda:dict(reason=first,start_deg=np.degrees(q).round(2).tolist(),
        target_mm_rpy_deg=list(goal[:6]),lift_rad=goal[6],limits_deg=np.degrees([lo,hi]).round(2).tolist()))
    # A finite local search runs even with no Mac. Include wrist and shoulder
    # seeds, and discard a result if cancellation or the measured start changed.
    started=time.monotonic()
    try:
        entry=ctx.entries.get('ik')
        if not entry:raise ValueError(first)
        deadline=started+min(10.,max(0.,18.-ctx.ik_search_spent));seen=set()
        for j,e,w in itertools.product((0.,7.,-7.),(6.,-6.,10.,-10.),(0.,7.,-7.)):
            hint=ctx.suggestion('ik',binding)
            if time.monotonic()>=deadline and not (hint and hint['action']=='seed' and tuple(hint['seed_deg']) not in seen):break
            if ctx.entries.get('ik') is not entry:raise ValueError('规划期间起姿或控制权已变化')
            offsets=[j,e,-1.5*e,e/2.,w,-w]
            candidates=([hint['seed_deg'],offsets] if hint and hint['action']=='seed' else [offsets])
            for offset in candidates:
                key=tuple(offset)
                if key in seen:continue
                seen.add(key)
                try:solution=seed(offset)
                except ValueError:continue
                if ctx._entry('ik',binding) is not entry:raise ValueError('规划期间起姿或控制权已变化')
                if hint and offset==hint.get('seed_deg'):ctx.applied('ik')
                ctx.end('ik')
                return solution
        raise ValueError(first)
    finally:ctx.ik_search_spent+=time.monotonic()-started


class ApproachClearanceError(ValueError):
    """Only a geometric clearance rejection may trigger approach replanning."""


def observation_route(start, goal, check, ctx):
    """Plan from the measured start, with both legs checked before dispatch."""
    import numpy as np
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    try:
        return [start, goal], [check(start, goal)]
    except ApproachClearanceError as error:
        if ctx is None:
            raise
        original = str(error)

    # Interpolate within the taught joint ranges, changing joint sequencing.
    groups = dict(elbow_first=(2, 3), shoulder_first=(0, 1), wrist_first=(4, 5))
    binding = tuple(np.round(np.r_[start, goal], 5))
    remaining = [(name, fraction) for fraction in (.6, .7, .8) for name in ROUTE_CHOICES]
    entry = None
    try:
        for attempt in range(len(remaining)):
            if attempt == 1:
                ctx.failed('route', binding, 1, lambda: dict(
                    reason=original, choices=list(ROUTE_CHOICES),
                    start_deg=np.degrees(start).round(2).tolist(),
                    goal_deg=np.degrees(goal).round(2).tolist()))
                entry = ctx.entries.get('route')
            if attempt:
                advice = ctx.suggestion('route', binding)
                if entry is None or ctx.entries.get('route') is not entry:
                    raise ValueError('预观察点规划期间起姿或控制权已变化')
                preferred = advice['action'] if advice else None
                index = next((i for i, (name, _) in enumerate(remaining) if name == preferred), 0)
            else:
                index = 0
            name, fraction = remaining.pop(index)
            weight = np.full(6, 1.-fraction)
            weight[list(groups[name])] = fraction
            via = start + weight * (goal-start)
            try:
                proofs = [check(start, via), check(via, goal)]
            except ApproachClearanceError:
                continue
            if entry is not None and ctx._entry('route', binding) is not entry:
                raise ValueError('预观察点规划期间起姿或控制权已变化')
            if attempt and advice and name==advice['action']:ctx.applied('route')
            return [start, via, goal], proofs
        raise ApproachClearanceError(original)
    finally:
        ctx.end('route')
