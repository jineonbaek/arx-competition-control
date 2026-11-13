"""Local floor estimates in odom, supported by recent independent surface samples."""
import numpy as np


class LocalFloor:
    def __init__(self):
        from pathlib import Path
        import json
        cfg=Path(__file__).resolve().parents[1]/'config/fixed_dual.json'
        data=json.loads(cfg.read_text()) if cfg.exists() else {}
        self.heights=data['optical_height_m'] if data.get('enabled') else {'head':1.19279,'bottom':.23}
        self.frames=[]
        self.last_fit=-np.inf
        self.planes={}
        self.tick=-np.inf
        self.grid=None

    def observe(self,source,t,origin,rays,ground=None):
        # Sensor heights only bound the search; classification uses a fitted local surface.
        nominal=origin[2]-self.heights[source]
        pool=rays if ground is None else np.vstack((rays,ground))
        p=pool[np.isfinite(pool).all(axis=1)&(abs(pool[:,2]-nominal)<.16)]
        if len(p):
            # Lowest sample in each small XY bin suppresses returns on top of low objects.
            order=np.argsort(p[:,2]);bins=np.floor(p[order,:2]/.09).astype(int)
            idx=np.unique(bins,axis=0,return_index=True)[1]
            self.frames.append((t,nominal,p[order[idx]]))
        self.tick=max(self.tick,t)
        self.frames=[f for f in self.frames if self.tick-f[0]<=2.5]
        if self.tick-self.last_fit<.45:return
        self.last_fit=self.tick
        self.planes={};self.grid=None
        if len(self.frames)<3 or max(f[0] for f in self.frames)-min(f[0] for f in self.frames)<.18:return
        pool=np.vstack([f[2] for f in self.frames]);order=np.argsort(pool[:,2])
        idx=np.unique(np.floor(pool[order,:2]/.09).astype(int),axis=0,return_index=True)[1]
        p=pool[order[idx]]
        if len(p)<40:return
        # A coarse consensus across the floor rejects small isolated flat surfaces.
        rng=np.random.default_rng(113);best=0;coarse=None
        for _ in range(40):
            tri=p[rng.choice(len(p),3,replace=False)]
            n=np.cross(tri[1]-tri[0],tri[2]-tri[0])
            if abs(n[2])<.001 or np.linalg.norm(n[:2])/abs(n[2])>.14:continue
            n/=n[2]
            residual=(p-tri[0])@n
            mask=abs(residual)<.022
            if mask.sum()>best:
                coarse=mask;best=int(mask.sum())
        if best<40 or best<.55*len(p):return
        a=np.c_[p[:,:2],np.ones(len(p))]
        coef=np.linalg.lstsq(a[coarse],p[coarse,2],rcond=None)[0]
        if abs(origin[:2]@coef[:2]+coef[2]-nominal)>.075 or np.min(np.linalg.eigvalsh(np.cov(p[coarse,:2].T)))<.065:return
        p=p[abs(a@coef-p[:,2])<.027]
        # Fit locally in an overlapped neighbourhood; do not extrapolate to an unsupported tile.
        self._fit_tiles(p)

    def _fit_tiles(self,p):
        # Fit all overlapped 3x3 neighbourhoods as a batch. Each real support
        # contributes to at most nine tiles; no extrapolation beyond observed floor.
        tiles=np.floor(p[:,:2]/.30).astype(int)
        keys,labels=np.unique(tiles,axis=0,return_inverse=True)
        anchor=keys.min(0)
        lut=np.full(tuple(keys.max(0)-anchor+1),-1,int)
        lut[tuple((keys-anchor).T)]=np.arange(len(keys))
        groups=[];samples=[]
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                pos=tiles-anchor+[dx,dy]
                valid=np.all((pos>=0)&(pos<lut.shape),axis=1)
                ii=np.flatnonzero(valid);jj=lut[tuple(pos[valid].T)]
                groups.append(jj[jj>=0]);samples.append(ii[jj>=0])
        g=np.concatenate(groups);ii=np.concatenate(samples)
        centres=(keys+.5)*.30
        x,y=(p[ii,:2]-centres[g]).T;z=p[ii,2]
        def sums(weight=None):
            return np.bincount(g,weights=weight,minlength=len(keys))
        count=sums()
        A=np.empty((len(keys),3,3))
        A[:,0,0]=sums(x*x);A[:,1,1]=sums(y*y);A[:,2,2]=count
        A[:,0,1]=A[:,1,0]=sums(x*y);A[:,0,2]=A[:,2,0]=sums(x);A[:,1,2]=A[:,2,1]=sums(y)
        eligible=(count>=9)&(np.linalg.det(A)>1e-6)
        for right in (True,False):
            for up in (True,False):
                eligible &= sums(((x>=0)==right)&((y>=0)==up))>0
        fit=np.zeros((len(keys),3))
        rhs=np.c_[sums(x*z),sums(y*z),sums(z)]
        fit[eligible]=np.linalg.solve(A[eligible],rhs[eligible,:,None])[:,:,0]
        err=abs(fit[g,0]*x+fit[g,1]*y+fit[g,2]-z)
        eligible &= (np.linalg.norm(fit[:,:2],axis=1)<=.14)&(sums(err>.023)<=.1*count)
        if not eligible.any():return
        self.grid_origin=anchor
        self.grid=np.full(tuple(lut.shape)+(3,),np.nan)
        valid_keys=keys[eligible];coeff=fit[eligible]
        coeff[:,2]-=np.sum(centres[eligible]*coeff[:,:2],axis=1)
        self.grid[tuple((valid_keys-anchor).T)]=coeff
        self.planes={tuple(k):True for k in valid_keys}

    def height(self,xy):
        xy=np.asarray(xy).reshape(-1,2)
        result=np.full(len(xy),np.nan)
        if self.grid is None or not len(xy):return result
        ids=np.floor(xy/.30).astype(int)-self.grid_origin
        ok=np.all((ids>=0)&(ids<self.grid.shape[:2]),axis=1)
        c=self.grid[tuple(ids[ok].T)]
        result[ok]=np.sum(xy[ok]*c[:,:2],axis=1)+c[:,2]
        return result
