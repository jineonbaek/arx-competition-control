"""One source-time budget for chassis odometry and temporary body free space.

650 ms is the existing mask budget, not an extrapolation or timestamp rewrite.
"""
MAX_AGE_NS=650_000_000
MOTION_MAX_AGE_NS=1_000_000_000
FUTURE_NS=100_000_000

def fresh(stamp,now,max_age_ns=MAX_AGE_NS):
    return -FUTURE_NS<=now-int(stamp)<=max_age_ns

def motion_fresh(stamp,now):
    return fresh(stamp,now,MOTION_MAX_AGE_NS)
