"""Read-only image overlays. Pixel-space evidence, never motor targets."""
import cv2,numpy as np
from battery_edges import detect_face,PerspectiveTracker
from rim_edges import detect_rim,RimTracker,line_angle_error
_RIM_TRACKERS={v:RimTracker() for v in ('d435','d405')}
_FACE_TRACKER=PerspectiveTracker()

def line(img,a,b,color,width=2):cv2.line(img,tuple(np.rint(a).astype(int)),tuple(np.rint(b).astype(int)),color,width,cv2.LINE_AA)
def label(img,text,p,color=(255,255,255)):
 p=tuple(np.rint(p).astype(int));cv2.putText(img,text,p,cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,0),3,cv2.LINE_AA);cv2.putText(img,text,p,cv2.FONT_HERSHEY_SIMPLEX,.45,color,1,cv2.LINE_AA)
def arrow(img,a,b,color):cv2.arrowedLine(img,tuple(np.rint(a).astype(int)),tuple(np.rint(b).astype(int)),color,2,cv2.LINE_AA,tipLength=.15)

def annotate(rgb,camera,captured_at=None,sequence=None):
 from agent.cabinet_preview_geometry import detect
 out=rgb[:,:,::-1].copy();h,w=rgb.shape[:2];hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
 result=dict(camera=camera,space='image_pixels',robot_motion=False,executable=False)
 cyan=(255,230,0);yellow=(0,230,255);magenta=(230,80,255)
 red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=60)).astype(np.uint8)
 n,lab,stats,centers=cv2.connectedComponentsWithStats(red,8)
 choices=[i for i in range(1,n) if stats[i,4]>500 and stats[i,2]>40 and stats[i,3]>25]
 det=detect(rgb);quad=None
 if det.get('status')=='complete':
  quad=np.array(det['inner_quad_px'])
  result['rim']=dict(status='four_edges',quad_px=quad.tolist(),source=det.get('detection_source','observed_red_ring'))
  center=quad.mean(0);left=(quad[0]+quad[3])/2;right=(quad[1]+quad[2])/2;top=(quad[0]+quad[1])/2
  arrow(out,center,top,cyan);label(out,'rim axis (image)',top+[4,16],cyan)
  if camera=='d435':
   target=left*.75+right*.25;cv2.drawMarker(out,tuple(target.astype(int)),yellow,cv2.MARKER_CROSS,22,2)
   label(out,'LEFT SLOT',target+[7,19],yellow)
   result['left_target_px']=target.tolist()
 elif choices:
  i=max(choices,key=lambda i:stats[i,4]);mask=(lab==i).astype(np.uint8)
  cs=[]
  result['rim']=dict(status='partial_visible_only',bbox=stats[i,:4].tolist(),hidden_edges='unknown')
  label(out,'RIM: hidden bottom not reconstructed',(12,47),cyan)
 else:result['rim']=dict(status='not_found');label(out,'RIM NOT FOUND',(12,47),cyan)
 rim_raw=detect_rim(rgb)
 rim_fit=_RIM_TRACKERS[camera].update(rim_raw,float(captured_at),sequence) if captured_at is not None else rim_raw
 if rim_fit:
  result['rim_outline']=rim_fit
  rq=rim_fit['quad_px']
  for j,name in enumerate(('top','right','bottom','left')):
   edge=rim_fit['edges'].get(name)
   if edge is None:continue
   points=np.array(edge['segment_px']) if rq is None else np.asarray([rq[j],rq[(j+1)%4]])
   color=(255,110,30) if name=='top' else cyan
   if edge.get('held_recent'):
    a,b=points;count=max(2,int(np.linalg.norm(b-a)/7))
    for k in range(0,count,2):line(out,a+(b-a)*k/count,a+(b-a)*min(k+1,count)/count,color)
   else:line(out,*points,color)
  label(out,'TOP reference',np.array(rim_fit['edges']['top']['segment_px'][0])+[0,-5],(255,110,30))
 if camera=='d405':
  raw=detect_face(rgb)
  face=_FACE_TRACKER.update(raw,float(captured_at),sequence) if captured_at is not None else raw
  if face:
   q=np.asarray(face['quad_px']);held=face.get('tracking')=='held_recent'
   for j,(a,b) in enumerate(zip(q,np.roll(q,-1,axis=0))):
    if held:
     count=max(2,int(np.linalg.norm(b-a)/7))
     for k in range(0,count,2):line(out,a+(b-a)*k/count,a+(b-a)*min(k+1,count)/count,yellow)
    else:line(out,a,b,yellow)
    label(out,str(j+1),a+[3,-5],magenta)
   front=np.asarray(face['front_px']);rear=np.asarray(face['rear_px'])
   arrow(out,rear,front,magenta)
   tracking=face.get('tracking','single_frame')
   label(out,'FACE: 4 lines / perspective / '+tracking,(12,h-38),yellow)
   label(out,'image fit only; no metric calibration',(12,h-20),magenta)
   result['battery']=dict(status='perspective_four_edge_fit',**face,robot_motion=False,executable=False)
   if rim_fit and not held and not rim_fit['edges']['top'].get('held_recent',False):
    ref=rim_fit['edges']['top']['segment_px'];err=line_angle_error(q[:2],ref)
    result['alignment']=dict(battery_edge=[1,2],reference='red_frame_top_outer_edge',angle_error_deg=err,target_error_deg=0.,space='image_angles',executable=False)
    label(out,'1-2 / TOP error: %+.2f deg'%err,(12,91))

   if choices and not held:
    i=max(choices,key=lambda i:stats[i,4]);rx,ry,rw,rh=map(int,stats[i,:4]);pairs=[]
    for yy in range(max(ry,int(front[1])-12),min(h,ry+rh,int(front[1])+13)):
     xs=np.flatnonzero(lab[yy]==i);runs=np.split(xs,np.flatnonzero(np.diff(xs)>1)+1) if len(xs) else []
     runs=[a for a in runs if len(a)>=4]
     if len(runs)>=2 and runs[-1][0]-runs[0][-1]>.3*rw:pairs.append((runs[0][-1],runs[-1][0]))
    if len(pairs)>=4:
     l,r=np.median(pairs,axis=0);mid=(l+r)/2;cross=np.array([mid,front[1]])
     line(out,[mid,front[1]-25],[mid,front[1]+25],cyan);arrow(out,front,cross,(255,255,255))
     dx=float(front[0]-mid);label(out,'front/rim dx: %.1f px'%dx,(12,67))
     result['comparison']=dict(front_minus_rim_center_x_px=dx,rim_inner_x_px=[float(l),float(r)],interpretation='image offset only, not world height or rotation')
  else:
   result['battery']=dict(status='four_edges_unavailable',metric_calibration=False)
   label(out,'BATTERY: four-edge fit unavailable',(12,h-38),yellow)
 label(out,camera.upper()+' | LIVE OBSERVATION',(12,23))
 return out,result
