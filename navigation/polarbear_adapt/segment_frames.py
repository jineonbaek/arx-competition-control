"""Bounded exact-stamp segmentation pairing, with original source geometry."""
from collections import Counter,OrderedDict
import math

class SegmentFrames:
 def __init__(self,limit=12):
  if limit<2:raise ValueError('At least two pending frames required')
  self.limit=limit;self.pairs={};self.origins=OrderedDict();self.completed=-1;self.counts=Counter()
 @staticmethod
 def origin(value):
  try:
   result=tuple(float(x) for x in value)
   return result if len(result)==3 and all(math.isfinite(x) for x in result) else None
  except (TypeError,ValueError):return None
 def remember(self,stamp,frame,origin):
  origin=self.origin(origin)
  if origin is None:self.counts['invalid_origin']+=1;return
  self.origins[stamp]=(frame,origin)
  while len(self.origins)>2*self.limit:self.origins.popitem(last=False)
 def add(self,stamp,frame,kind,message):
  if kind not in ('ground','obstacle'):raise ValueError('Invalid segmentation kind')
  if stamp<=self.completed:self.counts['late_or_duplicate']+=1;return
  item=self.pairs.setdefault(stamp,dict(frame=frame,parts={},waited=False))
  if item['frame']!=frame:self.counts['frame_mismatch']+=1;return
  item['parts'][kind]=message
  while len(self.pairs)>self.limit:self.pairs.pop(min(self.pairs));self.counts['overflow']+=1
 def drain(self,now,resolve,publish):
  for stamp in list(self.pairs):
   if now-stamp>580_000_000 or now-stamp< -100_000_000:
    self.pairs.pop(stamp);self.counts['expired_or_future']+=1
  for stamp in sorted(self.pairs):
   item=self.pairs.get(stamp)
   if item is None or len(item['parts'])!=2:continue
   if stamp>now:break
   cached=self.origins.get(stamp)
   origin=cached[1] if cached is not None and cached[0]==item['frame'] else None
   if origin is None:
    origin=self.origin(resolve(stamp,item['frame']))
    if origin is None:
     if not item['waited']:self.counts['tf_wait_frames']+=1;item['waited']=True
     break
    self.counts['tf_resolved']+=1
   else:self.counts['source_geometry_used']+=1
   publish(item['parts'],origin)
   self.counts['published']+=1
   if item['waited']:self.counts['retried']+=1
   self.completed=stamp
   for old in list(self.pairs):
    if old<=stamp:
     self.pairs.pop(old)
     if old<stamp:self.counts['incomplete_superseded']+=1
 def health(self):return dict(self.counts,pending=len(self.pairs),origins=len(self.origins))
