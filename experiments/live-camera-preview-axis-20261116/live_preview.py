"""Read-only live RGB-D diagnostic. This service cannot command the robot."""
import http.cookiejar,json,math,re,shutil,threading,time,urllib.request,uuid
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path

MUTATING_ROUTES=('/api/capture','/api/axis')
ROOT=Path('/home/arx/arx-competition-control')
WORK=Path('/home/arx/arx-place-hold-20261116')
HERE=Path(__file__).resolve().parent
LOCK=threading.Lock()

def paired(a,b):
    try:return math.isfinite(float(a)) and math.isfinite(float(b)) and abs(float(a)-float(b))<=.2
    except (TypeError,ValueError):return False

def base_report():
    return {'executable':False,'robot_motion':False,'mode':'只读诊断','d435':None,'d405':None}

def axis_summary(frames,consensus):
    result=consensus(frames)
    return {'mode':'D405 多帧只读诊断','frame_count':result['frame_count'] if 'frame_count' in result else len(frames),
            'tilt_deg':result['tilt_deg'],'max_deviation_deg':result.get('max_deviation_deg'),
            'axis_base':result.get('axis_base'),'physical_accuracy_verified':False,'executable':False,'robot_motion':False,
            'note':'侧面长轴候选；多帧一致不代表物理精度，不能作为运动指令'}

def capture_axis():
    import numpy as np
    from d405_axis import axis_consensus
    base='http://127.0.0.1:9983';op=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    op.open(base+'/',timeout=3).read()
    capture_dir=WORK/'reports'/('d405-axis-live-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]);capture_dir.mkdir(parents=True)
    frames=[]
    for i in range(3):
        req=urllib.request.Request(base+'/api/grasp-scene',data=json.dumps({'camera':'d405'}).encode(),headers={'Origin':base,'Content-Type':'application/json'})
        with op.open(req,timeout=22) as response:view=json.load(response)
        dest=capture_dir/('d405-%02d.npz'%i);shutil.copyfile(ROOT/'web/hardware/runtime/grasp-preview/frame.npz',dest)
        with np.load(dest,allow_pickle=False) as src:frame={k:src[k] for k in src.files}
        if str(frame['id'])!=str(view['id']) or str(frame['camera_id'])!='d405':raise ValueError('采集帧身份发生变化')
        frames.append(frame)
    out=axis_summary(frames,axis_consensus);out['capture_directory']=str(capture_dir);return out

def capture():
    import numpy as np
    from agent.cabinet_preview_geometry import localize,opening
    from d405_partial import estimate_vertical_alignment
    out=base_report();out['d435_calibration']='CAD 估计，未验证实机精度';out['d405_calibration']='离线候选，未验证实机精度'
    base='http://127.0.0.1:9983';op=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    op.open(base+'/',timeout=3).read()
    capture_dir=WORK/'reports'/('live-preview-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]);capture_dir.mkdir(parents=True)
    frames={}
    for name,cache in (('d435','grasp-head'),('d405','grasp-preview')):
        try:
            req=urllib.request.Request(base+'/api/grasp-scene',data=json.dumps({'camera':name}).encode(),headers={'Origin':base,'Content-Type':'application/json'})
            with op.open(req,timeout=22) as response:view=json.load(response)
            dest=capture_dir/(name+'.npz');shutil.copyfile(ROOT/'web/hardware/runtime'/cache/'frame.npz',dest)
            with np.load(dest,allow_pickle=False) as src:frame={k:src[k] for k in src.files}
            if str(frame['id'])!=str(view['id']) or str(frame['camera_id'])!=name:raise ValueError('采集帧身份发生变化')
            frames[name]=frame
        except Exception as exc:out[name]={'status':'unavailable','reason':str(exc)[:200]}
    if 'd435' in frames:
        try:
            slot=opening(json.loads((WORK/'config/cabinet-profiles.json').read_text()),'demo')
            value=localize(frames['d435'],profile=slot,now=float(frames['d435']['image_time']))
            out['d435']={k:value[k] for k in ('status','center_base_m','measured_size_mm','rim_plane_p90_mm')}
            out['d435']['image_time']=float(frames['d435']['image_time'])
        except Exception as exc:out['d435']={'status':'unavailable','reason':str(exc)[:200]}
    if 'd405' in frames:
        try:
            value=estimate_vertical_alignment(frames['d405']);out['d405']=value;out['d405']['image_time']=float(frames['d405']['image_time'])
        except Exception as exc:out['d405']={'status':'unavailable','reason':str(exc)[:200]}
    if len(frames)==2:
        a=float(frames['d435']['image_time']);b=float(frames['d405']['image_time'])
        out['camera_gap_s']=abs(a-b);out['time_paired']=paired(a,b)
    out['capture_directory']=str(capture_dir);return out

class Handler(BaseHTTPRequestHandler):
    def reply(self,code,data,content_type):
        self.send_response(code);self.send_header('Content-Type',content_type);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.send_header('X-Content-Type-Options','nosniff');self.end_headers();self.wfile.write(data)
    def do_GET(self):
        if self.path=='/':return self.reply(200,(HERE/'index.html').read_bytes(),'text/html; charset=utf-8')
        if self.path.split('?',1)[0]=='/app.js':return self.reply(200,(HERE/'app.js').read_bytes(),'application/javascript; charset=utf-8')
        name=self.path.split('?',1)[0].removeprefix('/camera/').removesuffix('.jpg')
        if self.path.startswith('/camera/') and name in ('d435','d405'):
            try:
                meta=json.loads((ROOT/'web/public/camera-live'/(name+'.json')).read_text());filename=meta['raw_image_file']
                if not re.fullmatch(name+r'-v-[a-f0-9]+-\d+\.jpg',filename):raise ValueError('invalid frame')
                return self.reply(200,(ROOT/'web/public/camera-live'/filename).read_bytes(),'image/jpeg')
            except (OSError,ValueError,KeyError):return self.reply(503,b'camera unavailable','text/plain')
        return self.reply(404,b'not found','text/plain')
    def do_POST(self):
        if self.path not in MUTATING_ROUTES:return self.reply(404,b'not found','text/plain')
        if self.headers.get('Origin')!='http://'+self.headers.get('Host',''):return self.reply(403,b'origin denied','text/plain')
        if not LOCK.acquire(blocking=False):return self.reply(409,b'capture busy','text/plain')
        try:
            data=json.dumps(capture_axis() if self.path=='/api/axis' else capture(),ensure_ascii=False,allow_nan=False).encode();return self.reply(200,data,'application/json; charset=utf-8')
        except Exception as exc:return self.reply(500,json.dumps({'error':str(exc)[:200],'executable':False},ensure_ascii=False).encode(),'application/json; charset=utf-8')
        finally:LOCK.release()

if __name__=='__main__':ThreadingHTTPServer(('127.0.0.1',9987),Handler).serve_forever()
