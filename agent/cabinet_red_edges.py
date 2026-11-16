"""Fit observed inner red edges even when the red ring is interrupted.

All four lines need actual distributed red-to-background transitions. Hidden
sections are extended from observed lines; a wholly absent edge is not guessed.
"""
import cv2
import numpy as np

def _occluded_quads(red,contours):
    """Use scanline inner transitions when an occluded corner changes the hull."""
    h,w=red.shape;found=[]
    def bands(values):
        a=np.flatnonzero(values)
        return np.split(a,np.flatnonzero(np.diff(a)>1)+1) if len(a) else []
    for contour in contours:
        x,y,bw,bh=cv2.boundingRect(contour)
        if min(x,y)<2 or x+bw>=w-1 or y+bh>=h-1 or min(bw,bh)<25:continue
        mask=np.zeros_like(red);cv2.drawContours(mask,[contour],-1,255,-1);mask&=red
        edges=[[],[],[],[]]
        for xx in range(x+int(.12*bw),x+int(.88*bw)):
            runs=bands(mask[y:y+bh,xx]>0)
            if not runs:continue
            if 3<=len(runs[0])<bh*.35:edges[0].append([xx,y+runs[0][-1]+.5])
            if len(runs)>1 and runs[-1][0]-runs[0][-1]>.3*bh and len(runs[-1])>=3:
                edges[2].append([xx,y+runs[-1][0]-.5])
        for yy in range(y+int(.12*bh),y+int(.88*bh)):
            runs=bands(mask[yy,x:x+bw]>0)
            if not runs:continue
            if 3<=len(runs[0])<bw*.25:edges[3].append([x+runs[0][-1]+.5,yy])
            if len(runs)>1 and runs[-1][0]-runs[0][-1]>.3*bw and len(runs[-1])>=3:
                edges[1].append([x+runs[-1][0]-.5,yy])
        lines=[];evidence=[]
        for i,points in enumerate(edges):
            points=np.asarray(points,np.float32)
            if len(points)<12:break
            span=float(np.ptp(points[:,0 if i%2==0 else 1]));extent=bw if i%2==0 else bh
            if span<max(12.,extent*.22):break
            # Rounded tape corners and occluder boundaries are not straight-rim
            # support. Require a distributed majority to agree on the line.
            best=None;rng=np.random.default_rng(3)
            for _ in range(100):
                a,b=points[rng.choice(len(points),2,replace=False)];axis=b-a;length=np.linalg.norm(axis)
                if length<8:continue
                good=np.abs(np.cross(points-a,axis/length))<1.
                if best is None or good.sum()>best.sum():best=good
            if best is None or best.sum()<12 or best.mean()<.65:break
            points=points[best];span=float(np.ptp(points[:,0 if i%2==0 else 1]))
            if span<max(12.,extent*.22):break
            vx,vy,px,py=cv2.fitLine(points,cv2.DIST_HUBER,0,.01,.01).reshape(-1)
            origin=np.array([px,py]);axis=np.array([vx,vy]);error=np.abs(np.cross(points-origin,axis));p90=float(np.quantile(error,.9))
            if p90>1.2:break
            lines.append((origin,axis));evidence.append(dict(support_pixels=len(points),span=span/extent,p90_error_px=p90))
        # Perspective does not preserve parallel sides. Convexity, observed
        # support, depth coplanarity and the CAD dimensions validate the frame.
        if len(lines)!=4 or abs(lines[0][1]@lines[2][1])<.9 or abs(lines[1][1]@lines[3][1])<.9:continue
        corners=[]
        for i in range(4):
            a,u=lines[(i-1)%4];b,v=lines[i];matrix=np.column_stack((u,-v))
            if abs(np.linalg.det(matrix))<.5:break
            corners.append(a+np.linalg.solve(matrix,b-a)[0]*u)
        if len(corners)!=4:continue
        q=np.asarray(corners)
        if np.any(q<[2,2]) or np.any(q>[w-2,h-2]) or not cv2.isContourConvex(q.astype(np.float32)):continue
        if not .4*bw*bh<cv2.contourArea(q.astype(np.float32))<bw*bh:continue
        found.append(dict(status='complete',inner_quad_px=q.tolist(),inner_contour_px=q.tolist(),
            outer_contour_px=cv2.convexHull(contour)[:,0,:].tolist(),observed_edges=4,ring_closed=False,
            detection_source='four_observed_scanline_edges',edge_evidence=evidence,_red=red))
    return found


def fit_open_red_edges(red):
    contours,_=cv2.findContours(red,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    candidates=[];h,w=red.shape
    for contour in contours:
        if cv2.contourArea(cv2.convexHull(contour))<500:continue
        x,y,bw,bh=cv2.boundingRect(contour)
        if min(x,y)<2 or x+bw>=w-1 or y+bh>=h-1:continue
        hull=cv2.convexHull(contour);q=None
        for epsilon in (.015,.02,.025,.035,.045):
            approx=cv2.approxPolyDP(hull,epsilon*cv2.arcLength(hull,True),True)
            if len(approx)==4:q=approx[:,0,:].astype(float);break
        if q is None:continue
        center=q.mean(0);q=q[np.argsort(np.arctan2(q[:,1]-center[1],q[:,0]-center[0]))]
        q=np.roll(q,-int(np.argmin(q.sum(1))),axis=0)
        lines=[];evidence=[]
        for a,b in zip(q,np.roll(q,-1,axis=0)):
            tangent=b-a;length=np.linalg.norm(tangent)
            if length<20:break
            tangent/=length;normal=np.array([-tangent[1],tangent[0]])
            if normal@(center-(a+b)/2)<0:normal=-normal
            extent=min(24.,min(bw,bh)*.28)
            offsets=np.arange(-2.,extent,.5);points=[];fractions=[]
            for f in np.linspace(.12,.88,max(20,int(length/2))):
                xy=a+f*(b-a)+offsets[:,None]*normal
                ij=np.rint(xy).astype(int)
                values=red[np.clip(ij[:,1],0,h-1),np.clip(ij[:,0],0,w-1)]>0
                # At least 2px of red before, 2px of background after.
                edges=[j for j in range(4,len(values)-4) if values[j-4:j].all() and not values[j:j+4].any()]
                if not edges:continue
                j=edges[0];points.append(a+f*(b-a)+(offsets[j]-.25)*normal);fractions.append(f)
            if len(points)<10 or np.ptp(fractions)<.3:break
            points=np.asarray(points,np.float32)
            vx,vy,px,py=cv2.fitLine(points,cv2.DIST_HUBER,0,.01,.01).reshape(-1)
            axis=np.array([vx,vy]);origin=np.array([px,py]);error=np.abs(np.cross(points-origin,axis))
            if abs(axis@tangent)<.985 or np.quantile(error,.9)>1.2:break
            lines.append((origin,axis));evidence.append(dict(support_pixels=len(points),span=float(np.ptp(fractions)),p90_error_px=float(np.quantile(error,.9))))
        if len(lines)!=4:continue
        corners=[]
        for i in range(4):
            a,u=lines[(i-1)%4];b,v=lines[i];matrix=np.column_stack((u,-v))
            if abs(np.linalg.det(matrix))<.1:break
            corners.append(a+np.linalg.solve(matrix,b-a)[0]*u)
        if len(corners)!=4:continue
        corners=np.asarray(corners);area=cv2.contourArea(corners.astype(np.float32))
        if not .35<area/cv2.contourArea(hull)<.96:continue
        if any(cv2.pointPolygonTest(hull,tuple(map(float,p)),True)<-1 for p in corners):continue
        if not cv2.isContourConvex(corners.astype(np.float32)):continue
        candidates.append(dict(status='complete',inner_quad_px=corners.tolist(),inner_contour_px=corners.tolist(),
            outer_contour_px=hull[:,0,:].tolist(),observed_edges=4,ring_closed=False,
            detection_source='visible_red_line_intersections',edge_evidence=evidence,_red=red))
    if not candidates:candidates=_occluded_quads(red,contours)
    if len(candidates)>1:return dict(status='ambiguous',reason='存在多个可拟合红框，未自动选择')
    return candidates[0] if candidates else None
