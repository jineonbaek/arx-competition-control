"""Pure sensor/goal guards for passive preview (standard library only)."""
import math

def level_points(points,q):
 x,y,z,w=q;n=math.sqrt(sum(v*v for v in q))
 if not math.isfinite(n) or n<1e-9:raise ValueError('invalid quaternion')
 x,y,z,w=[v/n for v in q]
 R=((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),(2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),(2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))
 return [[sum(R[i][j]*p[j] for j in range(3)) for i in range(3)] for p in points]

def obstacle_scan(points):
 r=[float('nan')]*720
 for x,y,z in points:
  d=math.hypot(x,y)
  if not all(math.isfinite(v) for v in (x,y,z)) or not .2<=d<=20 or not -.20<=z<=1.1:continue
  i=min(719,int((math.atan2(y,x)+math.pi)/(2*math.pi)*720))
  if not math.isfinite(r[i]) or d<r[i]:r[i]=d
 return r

def sanitize_ranges(ranges,minimum,maximum):
 return [float(r) if math.isfinite(r) and minimum<=r<=maximum else float('nan') for r in ranges]

def valid_goal(p,cloud_age,odom_age,frame):
 return frame=='preview_map' and all(math.isfinite(v) for v in p) and 0<=cloud_age<1. and 0<=odom_age<1.

def split_ground(points):
 """Only accept linefit ground near nominal floor; conserve ambiguous obstacles."""
 ground=[];obstacles=[]
 for p in points:
  if not all(math.isfinite(float(v)) for v in p):continue
  (ground if -.28<=p[2]<=-.20 else obstacles).append(p)
 return ground,obstacles
