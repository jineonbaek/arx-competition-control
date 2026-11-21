"""Retain an explicit route start through temporary input loss, never cancellation."""
import copy,time,os
from pathlib import Path
from urllib.error import URLError,HTTPError
from tf2_ros import TransformException
from mppi_trial.recovery import InputRecoveryWindow


def temporary_input(error):
 return (isinstance(error,TransformException) or
         isinstance(error,HTTPError) and error.code in (502,503,504) or
         isinstance(error,(TimeoutError,URLError)) and not isinstance(error,HTTPError) or
         str(error).startswith(('special_sensor_stale','special_pose_authority_invalid',
          'special_pose_stale','special_odometry_gap','special_source_repeated',
          'motion_pose_stale_or_invalid','motion_pose_changed_during_proof',
          'dual_origin_mapping_stale','bottom_obstacles_stale','定位未就绪','底盘反馈未就绪')))


class RouteStarter:
 def __init__(self,waypoints):
  self.w=waypoints;self.pending=None;self.console_epoch=None
 def cancel(self,reason=None):
  had=self.pending is not None;self.pending=None
  if had and reason:self.w.show('启动等待已取消：'+reason)
 def _context(self,expected_epoch=None):
  w=self.w;n=w.driver
  if w.blocking():raise ValueError('任务尚未归还控制权')
  console=w.runtime.console();epoch=console.get('command_epoch')
  if expected_epoch is not None and epoch!=expected_epoch:raise ValueError('控制指令已改变，请重新开始')
  self.console_epoch=epoch
  if not console['execution_enabled']:raise ValueError('执行已关闭，已撤销启动请求')
  if (console.get('chassis_scheduler') or {}).get('maintenance'):raise ValueError('底盘处于维护测试，已撤销启动请求')
  if not console['body']['fresh'] or console.get('chassis_active'):raise ValueError('底盘反馈未就绪或尚未停止')
  if n.ready_file is not None and not n.ready_file.is_file():raise ValueError('定位未就绪')
  return n.special_poses()
 def _dispatch(self,poses):
  w=self.w
  competition=bool(os.getenv('ARX_ADAPT_RUN') and (Path(os.environ['ARX_ADAPT_RUN'])/'competition-map').is_file())
  # Preserve the draft if validation or dispatch fails before a target is accepted.
  definition=w.route.definition()
  try:w.dispatch(w.route.start(w.hub.available_points(),pose=poses[0],direct_start=competition))
  except Exception:
   if w.driver.target is None:w.route.running=False;w.route.load(definition)
   raise
 def _reject(self,error):
  self.pending=None;self.w.show(('开始被拒绝：'+str(error)).replace('task worker unavailable:','任务程序未接入：'))
 def _waiting(self,reason):
  text='开始请求已保留，等待连续新扫描与有效定位：'+str(reason)
  if self.w.phase!=text:self.w.show(text)
 def request(self):
  w=self.w;n=w.driver
  if w.mapping_only:w.show('纯建图只编辑路线，请切换导航模式后执行');return
  if self.pending is not None:
   self.tick();return
  if w.route.running or n.target is not None:w.show();return
  if w.route.state!='draft' or not w.route.points:w.show();return
  self.console_epoch=None
  try:
   poses=self._context();self._dispatch(poses);w.show()
  except Exception as error:
   if not temporary_input(error) or self.console_epoch is None:self._reject(error);return
   self.pending=dict(definition=copy.deepcopy(w.route.definition()),seq=n.seq,
                     epoch=self.console_epoch,window=InputRecoveryWindow())
   self._waiting(error)
 def _revisions(self,poses):
  n=self.w.driver
  revisions=[]
  for topic in ('/pb/scan','/localization/odometry'):
   stamp=n.sensor.get(topic);revision=-1 if stamp is None else stamp.sec*1000000000+stamp.nanosec
   if topic=='/pb/scan':
    receiver=getattr(n,'tf_receiver',None);sample=None if receiver is None else receiver.scan_snapshot()
    if sample is not None and sample[0]>revision:revision=sample[0]
   revisions.append(revision)
  source=int(poses[2]*1e9)
  return tuple(revisions)+((source,),(source,))
 def tick(self):
  p=self.pending
  if p is None:return
  w=self.w;n=w.driver
  if n.seq!=p['seq'] or w.route.running or n.target is not None or w.route.state!='draft' or w.route.definition()!=p['definition']:
   self.cancel('导航已取消、接管或路线已改变');return
  try:
   poses=self._context(p['epoch'])
   if p['epoch'] is None:p['epoch']=self.console_epoch
   if not p['window'].observe(time.monotonic(),True,self._revisions(poses)):return
   self.pending=None
   try:self._dispatch(poses)
   except Exception as error:
    if temporary_input(error) and n.target is None:
     p['window']=InputRecoveryWindow();self.pending=p;self._waiting(error);return
    raise
   w.show()
  except Exception as error:
   if temporary_input(error):p['window'].observe(time.monotonic(),False);self._waiting(error)
   else:self._reject(error)
