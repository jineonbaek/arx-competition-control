"""In-memory execution/stop arbitration for offline validation. No hardware imports."""
from dataclasses import dataclass
import copy
import uuid
import threading
from functools import wraps
from .held_battery import finite
from .cabinet_gates import Decision


def synchronized(function):
    @wraps(function)
    def invoke(self,*args,**kwargs):
        with self.lock:return function(self,*args,**kwargs)
    return invoke


@dataclass(frozen=True)
class SimOwner:
    token:str
    actor:str
    parent_token:str|None=None


class SimulationExecutor:
    mode='offline'
    executable=False
    axes=('base','lift','right','left')
    def __init__(self,run_id,boot_id):
        if not run_id or not boot_id:raise ValueError('Simulation run/boot identities required')
        self.lock=threading.RLock();self.run_id=run_id;self.boot_id=boot_id;self.generation=0;self.owner=None;self.borrowers={}
        self.commands=[];self.segments={};self.state_seq=0;self.outcome_unknown=False;self.stopping=False;self.rest_verified=False
        self.stop_at=None;self.rest_rows=[];self.last_now=-float('inf');self.manual_latched=False

    def _clock(self,now):
        if not finite(now) or now<self.last_now:raise ValueError('Invalid/backward simulation clock')
        self.last_now=now

    @synchronized
    def acquire(self,actor,*,now):
        self._clock(now)
        if self.owner is not None or self.outcome_unknown or self.stopping:raise ValueError('Exclusive simulation owner/recovery already active')
        if not isinstance(actor,str) or not actor:raise ValueError('Simulation actor required')
        self.owner=SimOwner(uuid.uuid4().hex,actor);self.generation+=1;return self.owner

    @synchronized
    def borrow(self,owner,actor):
        if owner!=self.owner or owner is None:raise ValueError('Only active parent may lend control')
        child=SimOwner(uuid.uuid4().hex,actor,owner.token);self.borrowers[child.token]=child;return child

    def _owns(self,owner):
        return owner is not None and (owner==self.owner or self.borrowers.get(getattr(owner,'token',None))==owner)

    @synchronized
    def submit(self,segment_id,owner,decision,*,now):
        self._clock(now)
        if (not self._owns(owner) or self.stopping or self.manual_latched or self.outcome_unknown
                or not isinstance(decision,Decision) or not decision.allowed or decision.mode!='offline' or decision.executable is not False
                or decision.context.run_id!=self.run_id or decision.context.boot_id!=self.boot_id or decision.context.generation!=self.generation
                or decision.context.state_seq!=self.state_seq
                or not decision.issued_at<=now<decision.valid_until or decision.action in ('stop','inspect','observe','complete','release_progress')):
            raise ValueError('Stale/unowned/unqualified simulation intent')
        if not isinstance(segment_id,str) or not segment_id or segment_id in self.segments:raise ValueError('Segment cannot be replayed')
        if any(v['status']=='accepted' for v in self.segments.values()):raise ValueError('An earlier simulated segment is still in flight')
        row=dict(segment_id=segment_id,action=decision.action,status='accepted',generation=self.generation,
                 issued_at=now,expires_at=decision.valid_until,hard_deadline=now+30.,asset_hash=decision.context.asset_hash,
                 mode='offline',executable=False)
        self.segments[segment_id]=row;self.commands.append(copy.deepcopy(row));self.rest_verified=False
        return copy.deepcopy(row)

    @synchronized
    def tick(self,*,now):
        self._clock(now)
        if not self.stopping and any(r['status']=='accepted' and (now>=r['expires_at'] or now>=r['hard_deadline']) for r in self.segments.values()):
            self.request_stop('segment_evidence_expired',now=now)
        return dict(stopping=self.stopping,mode='offline',executable=False)

    @synchronized
    def renew(self,segment_id,owner,decision,*,now):
        self.tick(now=now);row=self.segments.get(segment_id)
        if (self.stopping or not self._owns(owner) or row is None or row['status']!='accepted'
                or not isinstance(decision,Decision) or not decision.allowed or decision.mode!='offline' or decision.executable is not False
                or decision.context.run_id!=self.run_id or decision.context.boot_id!=self.boot_id or decision.context.generation!=self.generation
                or decision.context.state_seq!=self.state_seq or decision.context.asset_hash!=row['asset_hash']
                or not decision.issued_at<=now<decision.valid_until
                or decision.action!=(('release_progress') if row['action']=='release' else row['action'])):
            raise ValueError('In-flight lease cannot be renewed from stale/unqualified evidence')
        row['expires_at']=min(decision.valid_until,row['hard_deadline'])

    @synchronized
    def receipt(self,segment_id,status,*,now):
        self.tick(now=now)
        if segment_id not in self.segments or status not in ('completed','unknown','rejected'):raise ValueError('Invalid simulated receipt')
        row=self.segments[segment_id]
        if row['status']!='accepted':raise ValueError('Terminal segment cannot be reinterpreted')
        row.update(status=status,receipt_at=now);self.state_seq+=1
        if status=='unknown':self.outcome_unknown=True;self.request_stop('outcome_unknown',now=now)
        return copy.deepcopy(row)

    @synchronized
    def request_stop(self,reason,*,now):
        self._clock(now)
        if not self.stopping:
            self.generation+=1;self.stopping=True;self.stop_at=now;self.rest_rows=[];self.rest_verified=False
        if reason in ('manual','takeover','cancel','operator_lease_lost'):self.manual_latched=True
        return dict(status='stopping',mode='offline',executable=False,stop_requested_axes=list(self.axes),
                    generation=self.generation,reason=reason,stopped_verified=self.rest_verified)

    @synchronized
    def observe_rest(self,row,*,now):
        self._clock(now)
        result=dict(stopped_verified=False,mode='offline',executable=False,generation=self.generation)
        try:
            if not self.stopping or row.get('scope')!='simulation' or row['generation']!=self.generation or set(row['axes'])!=set(self.axes):raise ValueError()
            self.rest_rows=[r for r in self.rest_rows if all(now-r['axes'][a]['captured_at']<=.5 for a in self.axes)]
            if len(self.rest_rows)<3:self.rest_verified=False
            if not isinstance(row['sample_id'],str) or not row['sample_id']:raise ValueError()
            if any(r['sample_id']==row['sample_id'] for r in self.rest_rows):return result|{'stopped_verified':self.rest_verified}
            previous=self.rest_rows[-1] if self.rest_rows else None
            if previous and any(row['axes'][a]['captured_at']-previous['axes'][a]['captured_at']>.5 for a in self.axes):
                self.rest_rows=[];self.rest_verified=False;previous=None
            for axis in self.axes:
                a=row['axes'][axis];limit=.005 if axis=='base' else .01
                if (type(a['seq']) is not int or not finite(a['captured_at']) or not finite(a['speed'])
                        or not self.stop_at<a['captured_at']<=now or now-a['captured_at']>.5 or abs(a['speed'])>limit):raise ValueError()
                if previous and (a['seq']<=previous['axes'][axis]['seq'] or a['captured_at']<=previous['axes'][axis]['captured_at']):return result
            self.rest_rows.append(copy.deepcopy(row))
            self.rest_rows=[r for r in self.rest_rows if all(now-r['axes'][a]['captured_at']<=.5 for a in self.axes)]
            first=self.rest_rows[0];last=self.rest_rows[-1]
            span=min(last['axes'][a]['captured_at']-first['axes'][a]['captured_at'] for a in self.axes)
            self.rest_verified=len(self.rest_rows)>=3 and span>=.2
            return result|{'stopped_verified':self.rest_verified}
        except (KeyError,TypeError,ValueError):self.rest_rows=[];self.rest_verified=False;return result

    @synchronized
    def release(self,owner):
        last_rest=min((v['captured_at'] for v in self.rest_rows[-1]['axes'].values()),default=-float('inf')) if self.rest_rows else -float('inf')
        if owner!=self.owner or owner is None or not self.stopping or not self.rest_verified or self.outcome_unknown or self.last_now-last_rest>.5:
            raise ValueError('Parent ownership requires verified stop and resolved outcomes')
        self.owner=None;self.borrowers={};self.generation+=1;self.stopping=False
