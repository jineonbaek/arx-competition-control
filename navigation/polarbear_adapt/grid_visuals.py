"""Exact occupancy-cell geometry. Same-color run merging, never decimation."""
import numpy as np

def cell_labels(values,width,height,costmap=False):
 if width<0 or height<0 or len(values)!=width*height:raise ValueError('invalid occupancy grid dimensions')
 a=np.asarray(values,dtype=np.int16).reshape(height,width)
 if ((a<-1)|(a>100)).any():raise ValueError('invalid occupancy values')
 labels=np.zeros(a.shape,np.uint8)
 if not costmap:labels[a==0]=1
 labels[a>=100]=2;labels[a==99]=3;labels[(a>0)&(a<99)]=4
 return labels

def merged_rectangles(labels):
 if not labels.size:return []
 result=[];active={}
 for y,row in enumerate(labels):
  edges=np.flatnonzero(np.r_[True,row[1:]!=row[:-1],True]);next_active={}
  for x0,x1 in zip(edges[:-1],edges[1:]):
   group=int(row[x0])
   if not group:continue
   key=(group,int(x0),int(x1));index=active.get(key)
   if index is None:index=len(result);result.append([group,int(x0),y,int(x1),y+1])
   else:result[index][4]=y+1
   next_active[key]=index
  active=next_active
 return result

def geometry(labels,rectangles,resolution):
 if not np.isfinite(resolution) or resolution<=0:raise ValueError('invalid map resolution')
 result={}
 for group in (1,2,3,4):
  rect=[r for r in rectangles if r[0]==group];count=int(np.count_nonzero(labels==group))
  if not count:result[group]=('triangles',np.empty((0,2)));continue
  if 6*len(rect)>=count:
   y,x=np.nonzero(labels==group);result[group]=('cubes',np.column_stack((x+.5,y+.5))*resolution);continue
  a=np.asarray(rect,dtype=float)[:,1:];x0,y0,x1,y1=a.T*resolution
  # CCW top-face triangles in the grid's own coordinates; origin pose stays on Marker.
  points=np.stack([np.column_stack((x0,y0)),np.column_stack((x1,y0)),np.column_stack((x1,y1)),np.column_stack((x0,y0)),np.column_stack((x1,y1)),np.column_stack((x0,y1))],axis=1).reshape(-1,2)
  result[group]=('triangles',points)
 return result
