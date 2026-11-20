"""Read-only image overlays. Pixel-space evidence, never motor targets."""
import cv2,numpy as np

def line(img,a,b,color,width=2):cv2.line(img,tuple(np.rint(a).astype(int)),tuple(np.rint(b).astype(int)),color,width,cv2.LINE_AA)
def label(img,text,p,color=(255,255,255)):
 p=tuple(np.rint(p).astype(int));cv2.putText(img,text,p,cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,0),3,cv2.LINE_AA);cv2.putText(img,text,p,cv2.FONT_HERSHEY_SIMPLEX,.45,color,1,cv2.LINE_AA)
def arrow(img,a,b,color):cv2.arrowedLine(img,tuple(np.rint(a).astype(int)),tuple(np.rint(b).astype(int)),color,2,cv2.LINE_AA,tipLength=.15)

def annotate(rgb,camera):
 from agent.cabinet_preview_geometry import detect
 out=rgb[:,:,::-1].copy();h,w=rgb.shape[:2];hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
 result=dict(camera=camera,space='image_pixels',robot_motion=False,executable=False)
 cyan=(255,230,0);yellow=(0,230,255);magenta=(230,80,255)
 red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=60)).astype(np.uint8)
 n,lab,stats,centers=cv2.connectedComponentsWithStats(red,8)
 choices=[i for i in range(1,n) if stats[i,4]>500 and stats[i,2]>40 and stats[i,3]>25]
 det=detect(rgb);quad=None
 if det.get('status')=='complete':
  quad=np.array(det['inner_quad_px']);cv2.polylines(out,[np.rint(quad).astype(np.int32)],True,cyan,2,cv2.LINE_AA)
  result['rim']=dict(status='four_edges',quad_px=quad.tolist(),source=det.get('detection_source','observed_red_ring'))
  center=quad.mean(0);left=(quad[0]+quad[3])/2;right=(quad[1]+quad[2])/2;top=(quad[0]+quad[1])/2
  arrow(out,center,top,cyan);label(out,'rim axis (image)',top+[4,16],cyan)
  if camera=='d435':
   target=left*.75+right*.25;cv2.drawMarker(out,tuple(target.astype(int)),yellow,cv2.MARKER_CROSS,22,2)
   label(out,'LEFT SLOT',target+[7,19],yellow)
   result['left_target_px']=target.tolist()
 elif choices:
  i=max(choices,key=lambda i:stats[i,4]);mask=(lab==i).astype(np.uint8)
  cs,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(out,cs,-1,cyan,2)
  result['rim']=dict(status='partial_visible_only',bbox=stats[i,:4].tolist(),hidden_edges='unknown')
  label(out,'RIM: partial; hidden edges unknown',(12,47),cyan)
 else:result['rim']=dict(status='not_found');label(out,'RIM NOT FOUND',(12,47),cyan)
 if camera=='d405':
  dark=((hsv[:,:,2]<125)&(hsv[:,:,1]<245)).astype(np.uint8)
  dark=cv2.morphologyEx(dark,cv2.MORPH_OPEN,np.ones((15,15),np.uint8))
  n,bodylab,bs,bc=cv2.connectedComponentsWithStats(dark,8)
  candidates=[i for i in range(1,n) if bs[i,4]>.025*h*w and bs[i,2]>.15*w]
  if candidates:
   i=min(candidates,key=lambda i:np.linalg.norm((bc[i]-[w*.5,h*.45])/[w,h]));mask=(bodylab==i).astype(np.uint8)
   x,y,bw,bh,_=map(int,bs[i]);rows=[]
   for yy in range(y,min(h,y+bh)):
    xx=np.flatnonzero(mask[yy]);
    if len(xx)>1:rows.append((yy,int(xx[0]),int(xx[-1])))
   spans=np.array([b-a for _,a,b in rows]);wide=[r for r in rows if r[2]-r[1]>.6*max(spans)]
   end=max(r[0] for r in wide) if wide else min(h-1,y+bh-1)
   green=((hsv[:,:,0]>65)&(hsv[:,:,0]<105)&(hsv[:,:,1]>35)&(hsv[:,:,2]>75)).astype(np.uint8)
   gn,gl,gs,gc=cv2.connectedComponentsWithStats(green,8)
   jaws=[j for j in range(1,gn) if gs[j,4]>.04*h*w and gs[j,1]+gs[j,3]>=h-3]
   cut=min([int(gs[j,1]) for j in jaws],default=end+1)
   end=min(end,cut-1);mask[end+1:]=0
   cs,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
   for contour in cs:
    vertices=contour[:,0,:]
    for a,b in zip(vertices,np.roll(vertices,-1,axis=0)):
     if not (a[1]>=end-1 and b[1]>=end-1):line(out,a,b,yellow)
   near=[r for r in rows if y+3<=r[0]<=y+13];near=near or rows[:5]
   front=np.array([np.median([(a+b)/2 for _,a,b in near]),np.median([yy for yy,_,_ in near])])
   rear=np.array([np.median([(a+b)/2 for yy,a,b in rows if max(y,end-25)<=yy<=end]),end])
   arrow(out,rear,front,magenta);label(out,'BODY visible above gripper',(12,h-38),yellow)
   label(out,'axis toward front',(12,h-20),magenta)
   result['battery']=dict(status='visible_upper_contour_candidate',front_px=front.tolist(),rear_px=rear.tolist(),occlusion_cut_y_px=end,occluded_contour='not reconstructed')
   # Compare at the front scanline; no conversion of pixels to mm.
   if choices:
    i=max(choices,key=lambda i:stats[i,4]);rx,ry,rw,rh=map(int,stats[i,:4]);pairs=[]
    for yy in range(max(ry,int(front[1])-12),min(h,ry+rh,int(front[1])+13)):
     xs=np.flatnonzero((lab[yy]==i));runs=np.split(xs,np.flatnonzero(np.diff(xs)>1)+1) if len(xs) else []
     runs=[a for a in runs if len(a)>=4]
     if len(runs)>=2 and runs[-1][0]-runs[0][-1]>.3*rw:pairs.append((runs[0][-1],runs[-1][0]))
    if len(pairs)>=4:
     l,r=np.median(pairs,axis=0);mid=(l+r)/2;cross=np.array([mid,front[1]])
     line(out,[mid,front[1]-30],[mid,front[1]+35],cyan);arrow(out,front,cross,(255,255,255))
     dx=float(front[0]-mid);label(out,'front/rim dx: %.1f px'%dx,(12,67))
     result['comparison']=dict(front_minus_rim_center_x_px=dx,rim_inner_x_px=[float(l),float(r)],interpretation='image offset only, not world height or rotation')
  else:result['battery']=dict(status='not_found')
 label(out,camera.upper()+' | LIVE OBSERVATION',(12,23))
 return out,result
