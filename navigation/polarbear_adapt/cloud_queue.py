"""Keep a few exact-stamped clouds until their matching TF becomes available."""
class PendingClouds:
    def __init__(self,limit=4):
        if limit<2:raise ValueError('TF wait queue needs at least two slots')
        self.limit=limit;self.items={};self.completed=-1
        self.stale_drops=0;self.overflow_drops=0
    def __len__(self):return len(self.items)
    def add(self,stamp,value):
        if stamp<=self.completed:return False
        self.items[stamp]=value
        while len(self.items)>self.limit:
            self.items.pop(min(self.items));self.overflow_drops+=1
        return True
    def take_ready(self,now,resolve):
        for stamp in list(self.items):
            if now-stamp>1_000_000_000 or now-stamp< -100_000_000:
                self.items.pop(stamp);self.stale_drops+=1
        for stamp in sorted(self.items,reverse=True):
            value=self.items[stamp];transforms=resolve(value)
            if transforms is None:continue
            for old in list(self.items):
                if old<=stamp:self.items.pop(old)
            self.completed=stamp
            return value,transforms
        return None
