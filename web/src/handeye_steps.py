import json
import secrets
import threading
import time
from pathlib import Path
import numpy as np
from handeye_column import ColumnModel
from handeye_observation import board_spec, observe

# Small wrist rotations relative to one measured session anchor, in degrees.
OFFSETS = [(0,0,0,0,0,8),(0,0,0,0,8,0),(0,0,0,0,0,-8),(0,0,0,0,-8,0),
           (0,0,0,6,0,5),(0,0,0,-6,0,-5),(0,0,0,0,10,8),(0,0,0,0,-10,8),
           (0,0,0,5,8,-6),(0,0,0,-5,-8,-6),(0,0,0,6,-6,8),(0,0,0,-6,6,8)]
OUT=Path(__file__).resolve().parents[1]/"hardware/runtime/handeye"

class Steps:
    def __init__(self, controller):
        self.ctrl=controller
        self.lock=threading.RLock()
        self.model=None
        self.session=None
        self.pending=None
        self.active=False
        self.cancelled=False
        self.history=[]
        self.anchor=None
        self.cursor=0
        self.spec=None
        self.owner=None

    def record(self, status, message, point=None):
        self.history.append({"point":point,"status":status,"message":message,"time":time.time()})
        self.history=self.history[-80:]

    def state(self,client):
        with self.lock:
            pending=dict(self.pending) if self.pending else None
            if pending and self.owner!=client:pending.pop("ticket",None)
            return {"session":self.session,"pending":pending,"active":self.active,"history":self.history[:],
                    "cursor":self.cursor,"total":len(OFFSETS)}

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
        with self.lock:
            op=data.get("op")
            if op=="cancel":
                if self.owner!=client:raise ValueError("请在原会话取消")
                self.pending=None
                self.cancelled=True
                if self.active:self.ctrl.submit({"type":"arm.stop","arm":"right"},client)
                return self.state(client)
            if self.active:raise ValueError("当前点仍在执行或采样")
            if self.model is None:self.model=ColumnModel()
            s,q,h=self.feedback()
            if op=="begin":
                if self.pending and self.owner!=client and time.time()<self.pending["expires"]:raise ValueError("另一页面有待确认点")
                spec=board_spec(data) if data.get("dictionary") else None
                self.owner=client;self.session=secrets.token_hex(12);self.spec=spec
                self.anchor=q.copy();self.cursor=0;self.history=[];self.pending=None
                self.record("准备","已读取右臂实际起始姿态")
            elif self.owner!=client or data.get("session")!=self.session:
                raise ValueError("会话已更换，请重新开始")
            if op in ("begin","next"):
                if self.pending:self.record("跳过","操作员跳过待确认点",self.pending["point"])
                self.pending=None
                while self.cursor<len(OFFSETS):
                    self.cursor+=1
                    target=self.anchor+np.radians(OFFSETS[self.cursor-1])
                    try:
                        check=self.assess(s,q,h,target)
                        if not check["clear"]:raise ValueError(f"整段间隙下界 {check['lower_mm']} mm，未超过 20 mm")
                    except ValueError as err:
                        self.record("跳过",str(err),self.cursor);continue
                    self.pending={"point":self.cursor,"ticket":secrets.token_urlsafe(18),"expires":time.time()+90,
                                  "start":q.tolist(),"target":target.tolist(),"lift":h,"check":check}
                    break
                return self.state(client)
            if op!="execute":raise ValueError("未知操作")
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
            self.active=True;self.cancelled=False
            try:
                self.ctrl.submit({"type":"arm.joints","arm":"right","joints":np.degrees(target).tolist()},client)
            except Exception:
                self.active=False
                try:self.ctrl.submit({"type":"arm.stop","arm":"right"},client)
                finally:self.record("无效","下发未确认，本点不重试",p["point"])
                raise
            self.record("执行中","右臂驱动已接受当前点，等待到位",p["point"])
            threading.Thread(target=self.finish,args=(client,p,target),daemon=True).start()
            return self.state(client)

    def finish(self, client,p,target):
        deadline=time.monotonic()+12
        stable=[];frames=[];reason="未到位或未停稳";settled=False
        try:
            while time.monotonic()<deadline:
                time.sleep(.08)
                with self.lock:
                    if self.cancelled:raise ValueError("操作员已停止")
                    s,q,h=self.feedback()
                    if abs(h-p["lift"])>.05:raise ValueError("升降反馈已变化")
                    if not self.assess(s,q,h,target)["clear"]:raise ValueError("实际路径重新检查未通过")
                    arm=s["arms"]["right"]
                    if np.max(abs(q-target))>np.radians(.5):
                        stable=[];frames=[];settled=False;continue
                    if not stable or arm["timestamp"]>stable[-1][0]:stable.append((arm["timestamp"],q.copy()))
                    stable=[x for x in stable if arm["timestamp"]-x[0]<.65]
                    if len(stable)<4 or stable[-1][0]-stable[0][0]<.40 or np.max(np.ptp([x[1] for x in stable],axis=0))>np.radians(.18):
                        frames=[];continue
                    settled=True
                    try:sample,image=observe(self.spec,stable[0][0],arm,self.model)
                    except (ValueError,OSError) as err:
                        reason=str(err);frames=[];continue
                    if frames and frames[-1][0]["camera_session"]!=sample["camera_session"]:frames=[]
                    if frames and frames[-1][0]["frame"]==sample["frame"]:continue
                    frames.append((sample,image));frames=frames[-3:]
                    if len(frames)<3:continue
                    transforms=np.array([f[0]["T_camera_from_marker"] for f in frames])
                    if np.max(np.linalg.norm(transforms[:,:3,3]-transforms[-1,:3,3],axis=1))>.005 or np.max(np.abs(transforms[:,:3,:3]-transforms[-1,:3,:3]))>.05:
                        reason="连续三帧标记位姿不稳定";continue
                    folder=OUT/self.session/f"point-{p['point']:02d}"
                    folder.mkdir(parents=True,exist_ok=True)
                    (folder/"color.jpg").write_bytes(image)
                    (folder/"sample.json").write_text(json.dumps(sample,indent=2))
                    self.record("可用",f"已保存图像及成对位姿，重投影 RMS {sample['rms_px']:.2f} px",p["point"])
                    return
            raise ValueError(reason)
        except Exception as err:
            with self.lock:
                if not self.cancelled:
                    try:self.ctrl.submit({"type":"arm.stop","arm":"right"},client)
                    except Exception as stop_err:self.record("停止未确认",str(stop_err),p["point"])
                self.record("不可用",str(err),p["point"])
        finally:
            with self.lock:self.active=False
