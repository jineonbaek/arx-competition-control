"""One source-time budget for chassis odometry and temporary body free space.

650 ms is the existing mask budget, not an extrapolation or timestamp rewrite.
"""
MAX_AGE_NS=650_000_000
FUTURE_NS=100_000_000

def fresh(stamp,now):
    return -FUTURE_NS<=now-int(stamp)<=MAX_AGE_NS
