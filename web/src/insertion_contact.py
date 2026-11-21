"""Position/current contact evidence. No commands and no calibrated force estimate."""
from collections import deque
import copy
import numpy as np


class InsertionContact(ValueError):
    def __init__(self, evidence):
        self.evidence = copy.deepcopy(evidence)
        super().__init__('检测到推进阻塞，等待停稳及坐底复核')


class ContactMonitor:
    fast_window_s = .5
    stall_window_s = 2.
    progress_mm = .25
    reference_lead_mm = .75
    tracking_growth_deg = .5
    max_age_s = .3

    def __init__(self, model, tcp, origin, axis, state, driver):
        self.model, self.tcp = model, np.asarray(tcp, float)
        self.origin, self.axis = np.asarray(origin, float), np.asarray(axis, float)
        q = np.asarray(state['arms']['right']['joints'], float)
        ref = np.asarray(driver.get('joint_reference_rad', q), float)
        if (q.shape != (6,) or ref.shape != (6,) or self.axis.shape != (3,)
                or not np.isfinite(np.r_[q, ref, self.axis]).all()
                or abs(np.linalg.norm(self.axis)-1.) > 1.e-6):
            raise ValueError('阻塞监测起始参考无效')
        # The executor preserves this load-supporting deflection in every node.
        self.bias = ref-q
        self.history = deque()
        self.currents = deque(maxlen=40)
        self.last_stamp = None
        self.last_learned_progress = 0.
        self.latest = None

    def settled(self, state):
        # Stationary encoders do not prove arrival when the position reference
        # still asks for forward motion. Keep observing through reference end.
        return self.latest is None or self.latest['reference_lead_mm'] < self.reference_lead_mm

    def sample(self, state, driver, now):
        arm = state['arms']['right']
        stamp = arm['received']
        if (not arm.get('fresh') or not np.isfinite([now, stamp]).all()
                or not 0. <= now-stamp <= self.max_age_s
                or self.last_stamp is not None and stamp < self.last_stamp):
            raise ValueError('阻塞监测关节反馈过期或倒退')
        if self.last_stamp == stamp:
            return
        self.last_stamp = stamp
        if driver.get('status') not in ('running', 'reference_complete'):
            self.history.clear()
            return
        q = np.asarray(arm['joints'], float)
        ref = np.asarray(driver.get('joint_reference_rad', q+self.bias), float)
        if q.shape != (6,) or ref.shape != (6,) or not np.isfinite(np.r_[q,ref]).all():
            raise ValueError('阻塞监测轨迹参考无效')
        height = state['body']['height']
        actual = self.model.wrist_transform(q,height) @ self.tcp
        commanded = self.model.wrist_transform(ref-self.bias,height) @ self.tcp
        progress = float((actual[:3,3]-self.origin[:3,3]) @ self.axis)*1000.
        lead = float((commanded[:3,3]-actual[:3,3]) @ self.axis)*1000.
        growth = np.abs(np.degrees(ref-q-self.bias))
        if not np.isfinite(np.r_[progress,lead,growth]).all():
            raise ValueError('阻塞监测轴向反馈无效')
        current = arm.get('currents')
        try:
            current = np.asarray(current, float)
            valid_current = current.shape == (6,) and np.isfinite(current).all()
        except (ValueError, TypeError):
            valid_current = False
        current_evidence = False
        baseline = threshold = delta = None
        if valid_current and len(self.currents) >= 8:
            data = np.abs(np.asarray(self.currents))
            baseline = np.median(data,axis=0)
            noise = np.median(np.abs(data-baseline),axis=0)
            threshold = np.maximum(np.maximum(6.*noise,.5*baseline),1.e-6)
            delta = np.abs(current)-baseline
            current_evidence = bool(np.any((delta > threshold) & (growth >= self.tracking_growth_deg)))
        # Do not learn a blockage into the load baseline. Only observed forward
        # motion with little additional position lead may update it.
        if valid_current and progress > self.last_learned_progress+.025 and lead < self.reference_lead_mm:
            self.currents.append(current.copy())
            self.last_learned_progress = progress
        row = dict(stamp=float(stamp),progress=progress,lead=lead,
                   tracking=float(np.max(growth)),current=current_evidence)
        self.history.append(row)
        while len(self.history)>1 and self.history[1]['stamp'] < stamp-self.stall_window_s-.1:
            self.history.popleft()
        self.latest = dict(progress_mm=progress,reference_lead_mm=lead,
                          tracking_growth_deg=row['tracking'],current_available=bool(valid_current))

        def stationary_window(seconds):
            rows=list(self.history)
            anchors=[i for i,r in enumerate(rows) if r['stamp'] <= stamp-seconds+1.e-9]
            if not anchors:
                return None
            rows=rows[anchors[-1]:]
            if (max(r['progress'] for r in rows)-min(r['progress'] for r in rows) > self.progress_mm
                    or min(r['lead'] for r in rows) < self.reference_lead_mm):
                return None
            return rows

        fast=stationary_window(self.fast_window_s)
        kind=None
        if fast and sum(r['current'] for r in fast) >= .8*len(fast):
            kind='current_contact'
        elif stationary_window(self.stall_window_s):
            kind='progress_stall'
        if kind:
            raise InsertionContact(dict(self.latest,kind=kind,sample_time=float(stamp),
                stop_confirmed=False,reference_reset=False,
                current_baseline=None if baseline is None else baseline.tolist(),
                current_delta=None if delta is None else delta.tolist(),
                current_threshold=None if threshold is None else threshold.tolist()))
