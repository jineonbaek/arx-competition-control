"""Finite, camera-verified insertion policy. Callbacks own cameras and actuation."""
import math,queue,threading,time


class ObservationUnavailable(ValueError):pass

def run_verified_insertion(observe,move,*,guard,event=lambda **v:None,cabinet_depth_mm=150.,
                           observation_timeout_s=2.5,observation_budget_s=8.,
                           max_compensations=2,max_compensation_mm=10.,max_extra_mm=20.):
    if not 100<=cabinet_depth_mm<=180:raise ValueError('仓深须为 100–180 mm')
    result=dict(status='insertion_unverified',insertion_depth_verified=False,cabinet_depth_mm=float(cabinet_depth_mm),
                initial_gap_mm=None,planned_initial_travel_mm=None,commanded_travel_mm=0.,compensation_count=0,
                compensation_mm=0.,verified_depth_mm=None,observations=[],reason='',push_started=False)
    spent=0.;reference=None;previous=None;last_command=0.
    def notify(phase,**values):
        result.update(values);event(phase=phase,**result)
    def read():
        nonlocal spent
        guard();remaining=min(observation_timeout_s,observation_budget_s-spent)
        if remaining<=0:raise ObservationUnavailable('入仓观测时间预算已用完')
        box=queue.Queue(maxsize=1)
        def worker():
            try:box.put((True,observe(reference)))
            except Exception as error:box.put((False,error))
        start=time.monotonic();threading.Thread(target=worker,daemon=True,name='insertion-observation').start()
        try:
            while True:
                guard();left=remaining-(time.monotonic()-start)
                if left<=0:raise ObservationUnavailable('入仓深度观测超时，已结束等待')
                try:ok,value=box.get(timeout=min(.03,left));break
                except queue.Empty:pass
            guard()
            if not ok:raise ObservationUnavailable(str(value))
            depth=value.get('depth_mm');stamp=value.get('image_time');ident=value.get('frame_id');cal=value.get('calibration_sha256')
            if (type(depth) not in (int,float) or not math.isfinite(depth) or not -300<=depth<=200
                or type(stamp) not in (int,float) or not math.isfinite(stamp) or not -.05<=time.time()-stamp<=2.
                or not isinstance(ident,str) or not ident or not isinstance(cal,str) or len(cal)!=64):
                raise ObservationUnavailable('入仓深度证据无效或过期')
            if previous and (stamp<=previous['image_time'] or ident==previous['frame_id'] or cal!=previous['calibration_sha256']):
                raise ObservationUnavailable('入仓复查不是同一标定的新曝光')
            result['observations'].append({k:value.get(k) for k in ['frame_id','image_time','depth_mm','support_pixels','capture_directory']})
            return value
        finally:spent+=time.monotonic()-start
    while True:
        notify('measure' if reference is None else 'verify')
        try:current=read()
        except ObservationUnavailable as error:
            result['reason']=str(error);break
        depth=float(current['depth_mm']);result['verified_depth_mm']=depth
        if reference is None:
            reference=current
            result.update(initial_gap_mm=max(0.,-depth),planned_initial_travel_mm=max(0.,cabinet_depth_mm-depth))
            if result['planned_initial_travel_mm']>300.:
                result['reason']='电池距仓口过远，本次未推入';break
        if previous and depth-previous['depth_mm']<min(.5,last_command*.20):
            result['reason']='推入后电池实测深度未增加，不继续补推';break
        remaining=cabinet_depth_mm-depth
        if -5.<=remaining<=2.:
            result.update(status='inserted_verified',insertion_depth_verified=True,reason='新深度帧确认电池前端达到仓内目标深度');break
        if remaining< -5.:
            result['reason']='观测深度超过目标，不能确认正常到位';break
        if previous is None:
            distance=remaining;notify('push')
        else:
            if result['compensation_count']>=max_compensations or result['compensation_mm']>=max_extra_mm:
                result['reason']='补偿已结束，入仓深度仍未确认';break
            distance=min(remaining,max_compensation_mm,max_extra_mm-result['compensation_mm'])
            result['compensation_count']+=1;result['compensation_mm']+=distance;notify('compensate')
        guard();move(float(distance),reference)
        result['push_started']=True;result['commanded_travel_mm']+=float(distance)
        previous=current;last_command=distance
    guard()
    result['observation_wait_s']=spent
    notify('verified' if result['insertion_depth_verified'] else 'unverified')
    guard()
    return result


def require_verified_receipt(receipt,depth_mm):
    verified=receipt.get('insertion_depth_verified') is True
    depth=receipt.get('verified_depth_mm')
    return bool(verified and type(depth) in (int,float) and math.isfinite(depth) and depth_mm-2.<=depth<=depth_mm+5.)
