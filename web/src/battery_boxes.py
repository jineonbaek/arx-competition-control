"""Whole-battery box filtering and rendering. No camera/robot control interfaces."""
import math
import cv2

def filter_boxes(rows,width,height,min_score=.30):
    output=[]
    for row in rows:
        try:
            coords=[float(v) for v in row['xyxy']];score=float(row['score'])
            if len(coords)!=4 or not all(math.isfinite(v) for v in coords+[score]) or int(row['class_id'])!=0 or score<min_score:continue
            x1,y1,x2,y2=coords
            if x2-x1<15 or y2-y1<15:continue
            partial=x1<=3 or y1<=3 or x2>=width-3 or y2>=height-3
            if not partial and score<max(min_score,.50):continue
            clipped=[max(0,min(width-1,int(round(x1)))),max(0,min(height-1,int(round(y1)))),max(0,min(width-1,int(round(x2)))),max(0,min(height-1,int(round(y2))))]
            if clipped[2]-clipped[0]<15 or clipped[3]-clipped[1]<15:continue
            output.append(dict(xyxy=clipped,score=score,partial=partial,xyz_m=None,label='battery',method='visual_reference_instance_detection'))
        except (ValueError,TypeError,KeyError,OverflowError):continue
    return sorted(output,key=lambda row:row['xyxy'][0])

def draw_boxes(image,boxes):
    result=image.copy()
    for i,box in enumerate(boxes,1):
        x1,y1,x2,y2=box['xyxy'];color=(0,180,255) if box['partial'] else (40,220,60)
        cv2.rectangle(result,(x1,y1),(x2,y2),color,2)
        label=f'BATTERY {i} {box["score"]:.2f}'+(' PARTIAL' if box['partial'] else '')
        cv2.putText(result,label,(x1+3,max(18,y1-7)),cv2.FONT_HERSHEY_SIMPLEX,.47,color,1,cv2.LINE_AA)
    cv2.rectangle(result,(0,0),(image.shape[1],42),(18,22,26),-1)
    text=f'WHOLE BATTERY: {len(boxes)} | VISUAL REFERENCE | READ ONLY'
    cv2.putText(result,text,(10,19),cv2.FONT_HERSHEY_SIMPLEX,.46,(240,240,240),1,cv2.LINE_AA)
    cv2.putText(result,'Orange: partially outside image | No 3D output',(10,35),cv2.FONT_HERSHEY_SIMPLEX,.40,(210,210,210),1,cv2.LINE_AA)
    return result
