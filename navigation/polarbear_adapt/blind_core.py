"""Blind policy measured from optical origin. Invalid returns never clear space."""
import math

def keep_point(point,minimum):
 return all(math.isfinite(float(v)) for v in point) and math.hypot(point[0],point[1])>=minimum

def filter_scan(ranges,minimum,maximum):
 return [float(v) if math.isfinite(v) and minimum<=v<=maximum else float('nan') for v in ranges]

def apply_occlusion(ranges,angle_min,angle_increment,near_angles,padding=1):
 """Do not replace masked near hits by farther returns at the same azimuth."""
 out=list(ranges);n=len(out)
 if near_angles is None:return [float('nan')]*n
 if not n:return out
 full=abs(n*angle_increment-2*math.pi)<2*angle_increment
 for a in near_angles:
  a=(float(a)-angle_min)%(2*math.pi)+angle_min
  index=int(math.floor((a-angle_min)/angle_increment))
  for j in range(index-padding,index+padding+1):
   if full:out[j%n]=float('nan')
   elif 0<=j<n:out[j]=float('nan')
 return out
