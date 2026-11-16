"""Last validated chassis writer, with identity-scoped cleanup. No hardware imports.

All state transitions and publication share the Controller's RLock. Identity is
not authority: source bindings only prevent one caller ending another's output.
"""
import copy
import math
import threading
import time

IDENTITY_FIELDS = frozenset(('source_id','task_id','generation','seq','issued_at','ttl_s'))


class InvalidChassis(ValueError):
    pass


def finite(v):
    return (type(v) is int and v.bit_length()<1024 or type(v) is float) and math.isfinite(v)


class ChassisScheduler:
    def __init__(self,output,*,clock=time.monotonic,wall=time.time,
                 output_clock=time.monotonic_ns,lease=.3,lock=None):
        self.output,self.clock,self.wall=output,clock,wall
        self.output_clock,self.lease=output_clock,lease
        self.lock=lock or threading.RLock()
        self.sources={};self.tasks={};self.active=None
        self.output_seq=0;self.maintenance=None;self.not_before=-math.inf

    def _identity(self,m,client):
        if not isinstance(m,dict) or not IDENTITY_FIELDS.issubset(m):
            raise InvalidChassis('底盘指令缺少任务身份或原始时效')
        for field in ('source_id','task_id'):
            if not isinstance(m[field],str) or not 1<=len(m[field])<=128:
                raise InvalidChassis('底盘来源或任务身份无效')
        if any(type(m[k]) is not int or not 0<m[k]<2**53 for k in ('generation','seq')):
            raise InvalidChassis('底盘代次或序号无效')
        if not finite(m['issued_at']) or not finite(m['ttl_s']) or not 0<m['ttl_s']<=self.lease:
            raise InvalidChassis('底盘指令时效无效')
        source=m['source_id']
        if source in self.sources and self.sources[source]!=client:
            raise InvalidChassis('底盘来源绑定了其他调用者')
        return source,m['task_id'],m['generation']

    def _emit(self,sdk,identity=None,*,expires_at=None,reason='command'):
        self.output_seq=max(self.output_seq+1,self.output_clock())
        issued=self.wall()
        expires=issued+self.lease if expires_at is None else expires_at
        if expires<=issued:raise InvalidChassis('底盘指令在输出前已过期')
        output=dict(output_seq=self.output_seq,issued_at=issued,
                    ttl_s=min(self.lease,expires-issued),sdk=list(sdk),reason=reason)
        if identity is not None:
            output.update(zip(('source_id','task_id','generation'),identity))
        self.output(output)
        return output

    def submit(self,m,client,sdk,*,maintenance=None):
        with self.lock:
            key=self._identity(m,client);base=key[:2]
            previous=self.tasks.get(base)
            end=m['type']=='chassis.end_output'
            if end:
                # Closing an older generation cannot affect a new generation.
                if previous and key[2]<previous['generation']:return dict(ended=True,stopped=False)
                self.sources[key[0]]=client
                self.tasks[base]=dict(generation=key[2],seq=m['seq'],ended=True)
                own=self.active is not None and self.active['identity']==key
                if own:
                    result=self._emit((0.,0.,0.),key,reason='end_output')
                    self.active=None
                else:result={}
                return dict(ended=True,stopped=own,**result)
            now=self.wall()
            if (m['issued_at']<self.not_before or m['issued_at']>now+.1
                    or now>=m['issued_at']+m['ttl_s']):
                raise InvalidChassis('底盘指令已过期或早于维护结束')
            if self.maintenance is not None and maintenance is not self.maintenance:
                raise InvalidChassis('维护测试独占期间拒绝普通底盘运动')
            if maintenance is not None and maintenance is not self.maintenance:
                raise InvalidChassis('维护测试身份已失效')
            if previous and (key[2]<previous['generation'] or key[2]==previous['generation']
                    and (previous['ended'] or m['seq']<=previous['seq'])):
                raise InvalidChassis('底盘任务已结束或指令序号已过期')
            if not isinstance(sdk,(tuple,list)) or len(sdk)!=3 or not all(finite(v) for v in sdk):
                raise InvalidChassis('SDK 执行值必须为三个有限数值')
            self.sources[key[0]]=client
            self.tasks[base]=dict(generation=key[2],seq=m['seq'],ended=False)
            result=self._emit(sdk,key,expires_at=m['issued_at']+m['ttl_s'])
            self.active=dict(identity=key,source_id=key[0],client=client,
                             deadline=self.clock()+result['ttl_s'],
                             output_seq=result['output_seq'],sdk=list(sdk))
            return result

    def stop(self,reason='explicit_stop'):
        with self.lock:
            for state in self.tasks.values():state['ended']=True
            self.not_before=self.wall()
            result=self._emit((0.,0.,0.),reason=reason)
            self.active=None
            return result

    def expire(self,output_seq):
        with self.lock:
            a=self.active
            if a is not None and a['output_seq']==output_seq:
                key=a['identity'];self.tasks[key[:2]]['ended']=True
                result=self._emit((0.,0.,0.),key,reason='timeout')
                self.active=None
                return result

    def tick(self,*,feedback_fresh=True):
        with self.lock:
            a=self.active
            if a is not None and any(a['sdk']) and (self.clock()>=a['deadline'] or not feedback_fresh):
                self.expire(a['output_seq'])

    def disconnect(self,client,identity=None):
        with self.lock:
            # A cookie/SSE stream is shared by tabs and refreshed pages. It is
            # not an output identity. Without one, the original watchdog owns
            # timeout cleanup; do not clear another page's fresh command.
            if identity is None:return
            key=tuple(identity)
            if len(key)!=3 or self.sources.get(key[0])!=client:return
            state=self.tasks.get(key[:2])
            if state is None or state['generation']!=key[2]:return
            state['ended']=True
            if self.active is not None and self.active['identity']==key:
                self.expire(self.active['output_seq'])

    def end_client(self,client):
        # Process-local work tokens are unique per job. Close only identities
        # already bound to that token; never infer ownership from a task label.
        with self.lock:
            identities=[(source,task,state['generation']) for (source,task),state in self.tasks.items()
                        if self.sources[source]==client and (not state['ended'] or
                            self.active is not None and self.active['identity']==(source,task,state['generation']))]
            for source,task,generation in identities:
                state=self.tasks[(source,task)]
                self.submit(dict(type='chassis.end_output',source_id=source,task_id=task,
                    generation=generation,seq=state['seq']+1,issued_at=self.wall(),ttl_s=self.lease),client,None)

    def reserve_maintenance(self):
        with self.lock:
            if self.maintenance is not None:raise InvalidChassis('已有维护测试')
            # Always fence known ordinary identities, including superseded ones.
            self.stop('maintenance_start');self.maintenance=object()
            return self.maintenance

    def release_maintenance(self,token):
        with self.lock:
            if token is None or token is not self.maintenance:raise InvalidChassis('维护测试身份已失效')
            self.stop('maintenance_end');self.not_before=self.wall();self.maintenance=None

    def snapshot(self):
        with self.lock:
            active=None if self.active is None else {k:v for k,v in self.active.items() if k!='client'}
            return dict(policy='last_valid_writer',active=copy.deepcopy(active),
                        output_seq=self.output_seq,maintenance=self.maintenance is not None)
