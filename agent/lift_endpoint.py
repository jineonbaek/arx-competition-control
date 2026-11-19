"""Pure reference proposals for a supervised lift endpoint experiment.

No transport or SDK objects. Desired measured height and SDK reference are
separate quantities; callers own new-feedback, ownership and stop supervision.
"""
import math


def settled_height(samples, *, target, window_s=1.5, max_span_rad=.005, max_age_s=.3):
    """Return the settled-window measurement furthest from the desired target.

    Arrival of this representative measurement implies the whole stable window
    fits the endpoint tolerance; a mean must not hide an outlying last packet.
    """
    if (type(target) not in (int,float) or not math.isfinite(target) or not 0<=target<=18
            or not samples or any(type(v) not in (int,float) or not math.isfinite(v) or v<=0
                          for v in (window_s,max_span_rad,max_age_s))):
        raise ValueError('Samples and positive finite settling policy required')
    packets=[];previous_now=None
    for sample in samples:
        height,stamp,now=(sample[k] for k in ('height','received','now'))
        if (any(type(v) not in (int,float) or not math.isfinite(v) for v in (height,stamp,now))
                or not 0<=height<=18 or not 0<=now-stamp<=max_age_s
                or (previous_now is not None and now<previous_now)):
            raise ValueError('Invalid or stale body feedback')
        if packets:
            last_stamp,last_height=packets[-1]
            if stamp<last_stamp or stamp-last_stamp>max_age_s:
                raise ValueError('Body feedback regressed or has a gap')
            if stamp==last_stamp:
                if height!=last_height:raise ValueError('Inconsistent cached body packet')
                previous_now=now
                continue
        packets.append((stamp,height));previous_now=now
    boundary=packets[-1][0]-window_s
    if len(packets)<2 or packets[0][0]>boundary:
        raise ValueError('Insufficient distinct body feedback duration')
    start=max(i for i,(stamp,_) in enumerate(packets) if stamp<=boundary)
    heights=[height for _,height in packets[start:]]
    if len(heights)<2 or max(heights)-min(heights)>max_span_rad:
        raise ValueError('Lift has not settled')
    return max(heights,key=lambda height:abs(height-target))


def next_reference(target, reference, measured, *, corrections,
                   tolerance=.02, max_correction=.30, max_step=.30,
                   max_corrections=3, reference_bounds=(0.,18.)):
    """Return arrived, correct, reference_limit or correction_limit.

    A correct result proposes one bounded command. Caller must re-measure after
    every command; reaching a reference never establishes measured arrival.
    """
    if not isinstance(reference_bounds,(tuple,list)) or len(reference_bounds)!=2:
        raise ValueError('Two reference bounds required')
    values=(target,reference,measured,tolerance,max_correction,max_step,*reference_bounds)
    if (any(type(v) not in (int,float) or not math.isfinite(v) for v in values)
            or not 0<=target<=18 or not 0<=measured<=18
            or not 0<=reference_bounds[0]<reference_bounds[1]<=18
            or not reference_bounds[0]<=reference<=reference_bounds[1]
            or min(tolerance,max_correction,max_step)<=0
            or type(corrections) is not int or type(max_corrections) is not int
            or max_corrections<1 or not 0<=corrections<=max_corrections):
        raise ValueError('Invalid bounded correction inputs')
    error=target-measured
    if abs(error)<=tolerance:
        return dict(status='arrived',reference=None,error=error)
    if corrections==max_corrections:
        return dict(status='correction_limit',reference=None,error=error)
    lower=max(reference_bounds[0],target-max_correction,reference-max_step)
    upper=min(reference_bounds[1],target+max_correction,reference+max_step)
    if lower>upper:
        return dict(status='reference_limit',reference=None,error=error)
    proposed=min(upper,max(lower,reference+error))
    if (proposed-reference)*error<=1e-9:
        return dict(status='reference_limit',reference=None,error=error)
    return dict(status='correct',reference=proposed,error=error)
