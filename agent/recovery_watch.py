"""Nonblocking fault reports and finite observation waits. No actuator commands."""
import logging
import time
from .codex_recovery import CURRENT, MAIL


def report_fault(stage,reason,*,run=None,evidence=None):
    try:
        ctx=CURRENT.get()
        mail=ctx.mail if ctx else MAIL
        return mail.diagnostic(str(run or (ctx.run if ctx else 'system')),str(stage),str(reason),evidence)
    except Exception:
        logging.getLogger(__name__).exception('Diagnostic report failed')
        return None


class RetryWatch:
    def __init__(self,run,stage,*,max_attempts=24,max_s=45.,clock=time.monotonic):
        self.run=run;self.stage=stage;self.max_attempts=max_attempts;self.max_s=max_s
        self.clock=clock;self.started=clock();self.attempts=0;self.best=None

    def reset(self):
        self.started=self.clock();self.attempts=0

    def progress(self,residual):
        value=abs(float(residual))
        if self.best is None or value<self.best-.2:
            self.best=value;self.reset()
        else:self.fail('有效测量未显示修正进展')

    def fail(self,reason,evidence=None):
        self.attempts+=1
        if self.attempts==3:report_fault(self.stage,reason,run=self.run,evidence=evidence)
        if self.attempts>=self.max_attempts or self.clock()-self.started>=self.max_s:
            detail=f'{self.stage} 连续 {self.attempts} 次未取得有效进展，不再原地重复；保持当前状态：{reason}'
            report_fault(self.stage,detail,run=self.run,evidence=evidence)
            raise ValueError(detail)
