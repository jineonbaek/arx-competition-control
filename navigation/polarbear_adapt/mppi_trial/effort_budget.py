"""SDK budgets: independent XY vector and yaw ceilings; Controller remains authoritative."""
import math
XY_LIMIT=.8
YAW_LIMIT=.8
def load(command):
 x,y,w=command
 if not all(math.isfinite(v) for v in command):return math.inf
 return max(math.hypot(x,y)/XY_LIMIT,abs(w)/YAW_LIMIT)
def bound(command):
 scale=1./max(1.,load(command))
 return tuple(v*scale for v in command)
