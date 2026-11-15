import {frameFresh,cameraImagePath} from './camera-freshness.js';
const el=id=>document.getElementById(id);
let busy=false,generation=0,currentMain=null;
const seen={},invalidSnapshots=new Set();
const phases=['D435主定位：彩色红框与CAD模型推算','仓外对齐高度与姿态（仅预览）','D405辅助：检查夹持与局部边缘，不要求完整仓口','沿水平线放入，Z与姿态保持不变','松爪与撤回尚未启用'];
for(const text of phases){const li=document.createElement('li');li.textContent=text;el('phases').appendChild(li);}
el('execute').disabled=true;el('stop-task').disabled=true;
async function api(path,body){
 const options={cache:'no-store',signal:AbortSignal.timeout(body?90000:5000)};
 if(body)Object.assign(options,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
 const response=await fetch(path,options),result=await response.json();
 if(!response.ok)throw Error(result.message||'请求失败');return result;
}
const key=r=>`${r.camera}:${r.model}:${r.snapshot_id}`;
function clearDetails(reason='本轮没有可用入仓路径'){
 currentMain=null;el('target').textContent=reason;
 for(const id of ['tolerance','model-info','depth-info','alignment-info','arm-info','calibration-info'])el(id).textContent='';
 el('blockers').replaceChildren();
}
function invalidateMain(reason){if(currentMain)invalidSnapshots.add(key(currentMain));clearDetails(reason);}
function expireDisplayed(){if(currentMain&&Date.now()/1000>currentMain.expires_at)invalidateMain('冻结预览已过期，请重新识别D435仓口。');}
const mm=p=>Array.isArray(p)?p.map(x=>Number(x).toFixed(1)).join(' / ')+' mm':'不可用';
function result(camera,r){
 if(!r||r.model!==el('cabinet-model').value)return;
 if(seen[camera]&&r.image_time<seen[camera])return;seen[camera]=r.image_time;
 const img=el(camera+'-result');
 if(typeof r.image==='string'&&r.image.startsWith('data:image/jpeg;base64,')){if(img.getAttribute('src')!==r.image)img.src=r.image;img.hidden=false;}
 el(camera+'-result-status').textContent=`${r.status} · ${new Date(r.image_time*1000).toLocaleTimeString()} · ${r.reason||'冻结帧 '+r.snapshot_id}`;
 if(camera!=='d435')return;
 const valid=r.validity?.valid===true&&Number.isFinite(r.expires_at)&&Date.now()/1000<=r.expires_at;
 if(!valid)invalidSnapshots.add(key(r));
 if(!r.plan||r.status!=='localized'||!valid||invalidSnapshots.has(key(r))){
  clearDetails('本轮没有可用入仓路径：'+(r.reason||(!valid?'预览过期或缺少状态核对':r.status)));
  el('calibration-info').textContent=r.calibration?.camera_status||'';return;
 }
 const p=r.plan,w=r.window;currentMain=r;
 el('target').textContent=`D435 · ${r.model==='competition'?'比赛上排中间仓口':'演示仓口'} · 左半区；模型预插入TCP ${mm(p.tcp_pre_base_mm)}，终点 ${mm(p.tcp_end_base_mm)}`;
 el('tolerance').textContent=`入仓段Z固定 ${p.fixed_z_mm.toFixed(1)} mm，Z变化 ${p.z_range_mm.toFixed(3)} mm；行程 ${p.stroke_mm.toFixed(1)} mm；模型最小净距 ${p.nominal_cad_clearance_mm.toFixed(2)} mm。`;
 el('model-info').textContent=`RGB与CAD净口 ${p.window_model_size_mm.map(x=>x.toFixed(1)).join(' × ')} mm 推算；重投影误差 ${w.reprojection_rms_px.toFixed(2)} px。${w.pose_ambiguous?'存在平面姿态歧义，不能当作实测精度。':''}`;
 el('calibration-info').textContent=`相机外参：${r.calibration?.camera_status||'状态未声明'}；沿用原抓取TCP。基座坐标精度尚未实测验收；点云与环境碰撞证据不可用。`;
 el('depth-info').textContent=`CAD内首个障碍深度 ${p.cad_first_obstacle_depth_mm.toFixed(1)} mm，预留后壁 ${p.back_margin_mm.toFixed(1)} mm；电池预计外露 ${p.battery_protrusion_mm.toFixed(1)} mm。`;
 el('alignment-info').textContent=r.alignment?.delta_z_mm!==undefined?`仓外需先对齐：相对当前TCP调整高度约 ${r.alignment.delta_z_mm.toFixed(1)} mm；随后固定Z入仓。`:'仓外对齐尚未完成，不能从当前姿态直接前伸。';
 el('arm-info').textContent=r.arm_path?.status==='kinematic_preview'?`已计算关节路径及立柱模型检查；TCP插值误差界 ${r.arm_path.interpolation_position_bound_mm.toFixed(3)} mm。${r.arm_path.outside_lift_alignment_required?'候选升降 '+r.arm_path.planning_lift_rad.toFixed(2)+' rad，双臂升降沿途尚未检查。':''}负载与环境路径尚未验收。`:`关节路径未通过：${r.arm_path?.reason||'待核对'}；蓝线仅为几何预览。`;
 el('blockers').replaceChildren(...p.blockers.map(text=>{const li=document.createElement('li');li.textContent=text;return li;}));
}
for(const c of ['d435','d405'])el('preview-'+c).onclick=async()=>{
 if(c==='d435')generation++;
 const requestGeneration=generation,model=el('cabinet-model').value;
 if(c==='d435')invalidateMain('正在重新识别D435主定位，旧路径已失效');
 busy=true;for(const k of ['d435','d405'])el('preview-'+k).disabled=true;
 el('status').textContent=c==='d435'?'正在更新D435彩色与模型定位…':'正在做D405局部辅助检查…';
 try{
  const r=await api('/api/cabinet-preview',{camera:c,model});
  if(requestGeneration!==generation)return;
  result(c,r);el('status').textContent=c==='d405'?'D405辅助结果已更新；D435主定位按时效单独核对':(r.reason||'D435预览已更新；实机执行保持禁用');
 }catch(e){
  if(requestGeneration!==generation)return;
  el(c+'-result').hidden=true;if(c==='d435')invalidateMain('D435主定位失败：'+e.message);el('status').textContent=e.message;
 }finally{busy=false;for(const k of ['d435','d405'])el('preview-'+k).disabled=false;expireDisplayed();}
};
el('cabinet-model').onchange=()=>{
 generation++;for(const c of ['d435','d405']){el(c+'-result').hidden=true;delete seen[c];}
 invalidateMain('模型已切换，请重新识别，不沿用旧落点或路径。');
};
async function refresh(){
 expireDisplayed();const requestGeneration=generation;
 try{
  const data=await api('/api/cabinet-preview');if(requestGeneration!==generation)return;
  if(!data.d435)invalidateMain('当前没有D435主定位，请重新识别。');
  for(const c of ['d435','d405'])result(c,data[c]);
 }catch(e){if(requestGeneration!==generation)return;invalidateMain('连接中断，旧路径已失效：'+e.message);el('status').textContent='连接中断：'+e.message;}
}
async function camera(c){
 try{const m=await api('/camera-live/'+c+'.json');if(!frameFresh(m,Date.now()))throw Error('stale');
  const img=el(c+'-live');img.src=cameraImagePath(c,m);img.hidden=false;el(c+'-live-status').textContent='实时彩色；路径来自单独冻结彩色帧与CAD模型';
 }catch{el(c+'-live').hidden=true;el(c+'-live-status').textContent='相机离线或画面已过期';}
}
setInterval(refresh,1500);setInterval(()=>{expireDisplayed();for(const c of ['d435','d405'])camera(c);},1000);refresh();
function executionResult(r){
 const active=['planning','queued','running','cancelling'].includes(r.status);
 el('execute').disabled=r.start_checks_available!==true||active||r.status==='recovery_required';
 el('execute').textContent='检查并执行右臂路径';el('stop-task').disabled=!active;
 el('stop-task').textContent='停止本次右臂任务';
 el('execution-status').textContent=`${r.status||'未连接'} · ${r.reason||'仓外到位容差5mm；入仓净空与实机证据检查保持有效，完成后保持夹持。'}`;
}
async function executionStatus(){
 try{executionResult(await api('/api/cabinet-execution'));}
 catch(e){el('execute').disabled=true;el('execution-status').textContent='任务状态连接中断：'+e.message;}
}
el('execute').onclick=async()=>{
 el('execute').disabled=true;
 try{executionResult(await api('/api/cabinet-execution',{op:'start',model:el('cabinet-model').value}));}
 catch(e){el('execution-status').textContent=e.message;}
};
el('stop-task').onclick=async()=>{
 try{executionResult(await api('/api/cabinet-execution',{op:'cancel'}));}
 catch(e){el('execution-status').textContent='停止请求结果未知：'+e.message;}
};
setInterval(executionStatus,1000);executionStatus();
