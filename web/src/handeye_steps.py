import json
import secrets
import threading
import time
from pathlib import Path
import numpy as np
from handeye_column import ColumnModel
from handeye_validation import check_point
from handeye_observation import LARGE_BOARD, LARGE_ROOT, inspect_large_board, WaitForFrame, board_spec, observe

# All points share the same raised J3/J4 targets, relative to the measured session start.
WRIST_OFFSETS = [(0,0), (0,8), (8,0), (0,-8), (-8,0), (6,5), (-6,-5),
                 (10,8), (-10,8), (8,-6), (-8,-6), (-6,8), (6,-8)]
# A supplementary round holds J1-J4 at the current pose and varies two wrist axes.
SUPPLEMENT_OFFSETS = [(10,0),(20,0),(16,10),(8,18),(0,20),(-10,16),
                      (-18,8),(-20,0),(-16,-10),(-8,-18),(0,-20),(10,-15),(18,-7)]
OUT=Path(__file__).resolve().parents[1]/"hardware/runtime/handeye"

def sample_folder(spec, session, point):
    root = LARGE_ROOT / "sessions" if spec.get("layout") == LARGE_BOARD["layout"] else OUT
    return root / session / f"point-{point:02d}"

def capture_position(start, target, actual, *, motion):
    """Sampling proximity only. Motion limits and collision checks are independent."""
    travel_deg = float(np.max(np.abs(np.degrees(target - start))))
    errors_deg = np.abs(np.degrees(target - actual))
    worst = int(np.argmax(errors_deg))
    tolerance = min(2.0, 0.25 * travel_deg) if motion else 0.5
    eligible = (not motion or travel_deg >= 0.5) and errors_deg[worst] <= tolerance
    return bool(eligible), worst + 1, float(errors_deg[worst])


class Steps:
    def __init__(self, controller, depth_control):
        self.depth_control=depth_control
        self.ctrl=controller
        self.lock=threading.RLock()
        self.model=None
        self.session=None
        self.pending=None
        self.active=False
        self.cancelled=False
        self.motion=False
        self.history=[]
        self.anchor=None
        self.cursor=0
        self.raised=False
        self.profile='raise'
        self.offsets=WRIST_OFFSETS
        self.spec=None
        self.owner=None

    def record(self, status, message, point=None):
        self.history.append({"point":point,"status":status,"message":message,"time":time.time()})
        self.history=self.history[-80:]

    def state(self,client):
        with self.lock:
            pending=dict(self.pending) if self.pending else None
            if pending and self.owner!=client:pending.pop("ticket",None)
            return {"time":time.time(),"session":self.session,"pending":pending,"active":self.active,"history":self.history[:],
                    "layout":(self.spec or {}).get("layout",""),"board_configured":self.spec is not None,"cursor":self.cursor,"total":len(self.offsets),"profile":self.profile}

    def feedback(self):
        s=self.ctrl.snapshot()
        if not s["ros"]["connected"] or s["chassis_active"]:raise ValueError("ROS 未连接或底盘正在运动")
        for p in [s["arms"]["right"],s["body"]]:
            if not p.get("fresh") or p.get("age_s",99)>.3:raise ValueError("右臂或升降反馈缺失、过期")
        q=np.array(s["arms"]["right"]["joints"])
        h=s["body"]["height"]
        if q.shape!=(6,) or not np.isfinite(np.r_[q,h]).all():raise ValueError("反馈数值无效")
        return s,q,h

    def assess(self,s,q,h,target):
        limits=np.radians(s["limits"]["arms"]["right"]["joint_limits_deg"])
        if np.any(np.minimum(q,target)<limits[:,0]) or np.any(np.maximum(q,target)>limits[:,1]):
            raise ValueError("超出右臂软件限位")
        if min(q[0],target[0]) < np.radians(-35.8):raise ValueError("超出右臂 J1 现场下限 −35.8°")
        return self.model.check(q,target,h)

    def wrist_rise_mm(self,q,h):
        start=self.model.pose(self.anchor,h)[1]
        wrist=self.model.pose(q,h)[1]
        return float(wrist[2,3]-start[2,3])*1000

    def manual_submit(self, data, client):
        with self.lock:
            kind=data.get("type") if isinstance(data,dict) else None
            if self.active and kind not in ("arm.stop","lift.stop","chassis.stop"):
                raise ValueError("标定单点执行中；请先停止本点")
            if self.active and kind=="arm.stop" and data.get("arm")=="right":
                self.cancelled=True
            return self.ctrl.submit(data,client)

    def action(self, client, data):
        if not isinstance(data,dict):raise ValueError("需要标定操作对象")
        if data.get("op")=="inspect_large_board":
            return inspect_large_board()
        if data.get("op")=="verify_point":
            def idle_feedback():
                with self.lock:
                    if self.active or self.pending:
                        raise ValueError("请先完成或取消当前标定点")
                    return self.feedback()
            with self.lock:
                if self.model is None:self.model=ColumnModel()
            return check_point(idle_feedback,self.model,data.get("reference_mm"))
        with self.lock:
            op=data.get("op")
            if op=="cancel":
                if self.owner!=client:raise ValueError("请在原会话取消")
                self.pending=None
                self.cancelled=True
                if self.active and self.motion:self.ctrl.submit({"type":"arm.stop","arm":"right"},client)
                return self.state(client)
            if self.active:raise ValueError("当前点仍在执行或采样")
            if self.model is None:self.model=ColumnModel()
            s,q,h=self.feedback()
            if op in ("begin","probe"):
                if self.pending and self.owner!=client and time.time()<self.pending["expires"]:raise ValueError("另一页面有待确认点")
                spec=board_spec(data) if data.get("dictionary") else None
                profile=data.get("profile","raise")
                if profile not in ("raise","supplement"):raise ValueError("未知候选姿态组")
                self.profile=profile
                self.offsets=SUPPLEMENT_OFFSETS if profile=="supplement" else WRIST_OFFSETS
                self.owner=client;self.session=secrets.token_hex(12);self.spec=spec
                self.anchor=q.copy();self.cursor=0;self.raised=profile=='supplement';self.history=[];self.pending=None
                self.record("准备","已读取右臂实际起始姿态")
            elif self.owner!=client or data.get("session")!=self.session:
                raise ValueError("会话已更换，请重新开始")
            if op=="probe":
                if not self.spec:raise ValueError("请先确认字典、标记编号和黑框边长")
                token=self.depth_control.acquire_handeye(False)
                self.active=True;self.motion=False;self.cancelled=False
                self.record("静止试采","原地检查标记与位姿，无运动指令",0)
                self.start_worker(client,{"point":0,"lift":h},q.copy(),token)
                return self.state(client)
            if op in ("begin","next"):
                if not self.raised:self.cursor=0
                elif self.pending:self.record("跳过","操作员跳过待确认点",self.pending["point"])
                self.pending=None
                while self.cursor<len(self.offsets):
                    self.cursor+=1
                    j5,j6=self.offsets[self.cursor-1]
                    lift_j3,lift_j4=(5,3) if self.profile=='raise' else (0,0)
                    target=self.anchor+np.radians([0,0,lift_j3,lift_j4,j5,j6])
                    try:
                        dz_mm=self.wrist_rise_mm(target,h)
                        if self.profile=='raise' and dz_mm<=0:raise ValueError("此起姿下模型未预测到腕部抬高")
                        check=self.assess(s,q,h,target)
                        if not check["clear"]:raise ValueError(f"整段间隙下界 {check['lower_mm']} mm，未超过 20 mm")
                    except ValueError as err:
                        self.record("跳过",str(err),self.cursor)
                        if not self.raised:break
                        continue
                    self.pending={"point":self.cursor,"ticket":secrets.token_urlsafe(18),"expires":time.time()+90,
                                  "start":q.tolist(),"target":target.tolist(),"lift":h,"check":check,
                                  "profile":self.profile,"wrist_rise_mm":round(dz_mm,1)}
                    break
                return self.state(client)
            if op!="execute":raise ValueError("未知操作")
            if self.spec is None:raise ValueError("请先填写标记参数并重新生成候选")
            p=self.pending
            if not p or data.get("ticket")!=p["ticket"] or time.time()>p["expires"]:
                raise ValueError("预览已失效，请重新规划")
            if data.get("confirmed") is not True:raise ValueError("请确认本次执行")
            self.pending=None
            target=np.array(p["target"])
            if np.max(np.abs(q-p["start"]))>np.radians(.5) or abs(h-p["lift"])>.05 or not self.assess(s,q,h,target)["clear"]:
                raise ValueError("起点或路径检查已变化，请重新规划")
            stamp=s["arms"]["right"]["timestamp"]
            time.sleep(.18)
            s2,q2,h2=self.feedback()
            if s2["arms"]["right"]["timestamp"]<=stamp or np.max(abs(q2-q))>np.radians(.15) or abs(h2-h)>.03 or not self.assess(s2,q2,h2,target)["clear"]:
                raise ValueError("机械臂或升降尚未停稳，请重新预览")
            token=self.depth_control.acquire_handeye(False)
            p=dict(p, start=q2.tolist())
            self.active=True;self.motion=True;self.cancelled=False
            try:
                self.ctrl.submit({"type":"arm.joints","arm":"right","joints":np.degrees(target).tolist()},client)
            except Exception:
                self.active=False
                try:self.ctrl.submit({"type":"arm.stop","arm":"right"},client)
                finally:
                    self.record("无效","下发未确认，本点不重试",p["point"])
                    self.release_depth(token)
                raise
            self.record("执行中","右臂驱动已接受当前点，等待到位",p["point"])
            self.start_worker(client,p,target,token)
            return self.state(client)

    def release_depth(self, token):
        try:
            self.depth_control.release_handeye(token)
        finally:
            self.active=False
            self.motion=False

    def start_worker(self,client,p,target,token):
        try:
            threading.Thread(target=self.finish,args=(client,p,target,token),daemon=True).start()
        except Exception:
            try:
                if self.motion:self.ctrl.submit({"type":"arm.stop","arm":"right"},client)
            finally:self.release_depth(token)
            raise

    def finish(self, client,p,target,token):
        deadline=time.monotonic()+12
        stable=[];frames=[];reason="等待新鲜反馈";position_note=""
        try:
            while time.monotonic()<deadline:
                time.sleep(.08)
                with self.lock:
                    if self.cancelled:raise ValueError("操作员已停止")
                    s,q,h=self.feedback()
                    if abs(h-p["lift"])>.05:raise ValueError("升降反馈已变化")
                    if not self.assess(s,q,h,target)["clear"]:raise ValueError("实际路径重新检查未通过")
                    arm=s["arms"]["right"]
                    start=np.asarray(p["start"]) if self.motion else target
                    near,joint,error=capture_position(start,target,q,motion=self.motion)
                    position_note=f"J{joint} 目标 {np.degrees(target[joint-1]):.2f}°，实测 {np.degrees(q[joint-1]):.2f}°，差 {error:.2f}°"
                    if not near:
                        reason="未进入参考姿态附近"
                        stable=[];frames=[];continue
                    if not stable or arm["timestamp"]>stable[-1][0]:stable.append((arm["timestamp"],q.copy()))
                    stable=[x for x in stable if arm["timestamp"]-x[0]<.65]
                    if len(stable)<4 or stable[-1][0]-stable[0][0]<.40:
                        reason="等待连续停稳反馈";frames=[];continue
                    jitter=float(np.max(np.ptp(np.degrees([x[1] for x in stable]),axis=0)))
                    if jitter>.18 or np.max(np.diff([x[0] for x in stable]))>.25:
                        reason=f"停稳窗口反馈间隔过长或关节波动 {jitter:.2f}°";frames=[];continue
                    if p["point"]==1 and not self.raised:
                        measured_rise=self.wrist_rise_mm(q,h)
                        if measured_rise<=0:raise ValueError("停稳后实测关节未形成腕部抬升，请检查现场")
                        self.raised=True
                        self.record("已抬腕",f"实测关节换算腕部抬高 {measured_rise:.1f} mm",1)
                    try:sample,image=observe(self.spec,stable[0][0],arm,self.model,stable)
                    except WaitForFrame:
                        if not frames and reason.startswith(("等待连续","等待新鲜","未进入","停稳窗口")):
                            reason="已接近参考姿态并停稳，等待相机与关节反馈配对"
                        continue
                    except (ValueError,OSError) as err:
                        reason=str(err);frames=[];continue
                    if frames and (frames[-1][0]["camera_session"]!=sample["camera_session"] or sample["image_time"]-frames[-1][0]["image_time"]>.65):frames=[]
                    if frames and frames[-1][0]["frame"]==sample["frame"]:continue
                    frames.append((sample,image));frames=frames[-3:]
                    if len(frames)<3:
                        reason=f"已停稳，标记有效帧 {len(frames)}/3";continue
                    transforms=np.array([f[0]["T_camera_from_marker"] for f in frames])
                    if np.max(np.linalg.norm(transforms[:,:3,3]-transforms[-1,:3,3],axis=1))>.005 or np.max(np.abs(transforms[:,:3,:3]-transforms[-1,:3,:3]))>.05:
                        reason="连续三帧标记位姿不稳定";continue
                    measured=np.array([f[0]["joints_rad"] for f in frames])
                    if np.max(np.ptp(measured,axis=0))>np.radians(.18):
                        reason="三帧对应的实测关节仍在移动";continue
                    sample["reference_joints_rad"]=target.tolist()
                    sample["max_reference_error_deg"]=float(np.max(np.abs(np.degrees(measured[-1]-target))))
                    folder=sample_folder(self.spec,self.session,p["point"])
                    folder.mkdir(parents=True,exist_ok=False)
                    (folder/"color.jpg").write_bytes(image)
                    (folder/"sample.json").write_text(json.dumps(sample,indent=2))
                    tag_summary=f"整板 {len(sample['inlier_marker_ids'])} 个有效标记，" if self.spec.get("layout") else ""
                    self.record("可用",f"{tag_summary}已保存图像及成对位姿，重投影 RMS {sample['rms_px']:.2f} px，按实测姿态保存（最大参考偏差 {sample['max_reference_error_deg']:.2f}°）",p["point"])
                    return
            raise ValueError(f"{reason}；{position_note}" if position_note else reason)
        except Exception as err:
            with self.lock:
                if self.motion and not self.cancelled:
                    try:self.ctrl.submit({"type":"arm.stop","arm":"right"},client)
                    except Exception as stop_err:self.record("停止未确认",str(stop_err),p["point"])
                self.record("不可用",str(err),p["point"])
        finally:
            with self.lock:self.release_depth(token)
