import {frameFresh,cameraImagePath} from './camera-freshness.js';
const el=id=>document.getElementById(id);let busy=false,activeCamera='d435';const seen={};
const phases=['D435识别红色仓口与前沿深度','仓外对齐高度与姿态（仅预览）','D405近距离复核完整仓口','沿水平线放入，Z与姿态保持不变','松爪与撤回尚未启用'];
for(const text of phases){const li=document.createElement('li');li.textContent=text;el('phases').appendChild(li);}
el('execute').disabled=true;el('stop-task').disabled=true;
async function api(path,body){const options={cache:'no-store',signal:AbortSignal.timeout(body?90000:5000)};if(body)Object.assign(options,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const response=await fetch(path,options);const result=await response.json();if(!response.ok)throw Error(result.message||'请求失败');return result;}
function clearDetails(reason='本轮没有可用入仓路径'){el('target').textContent=reason;for(const id of ['tolerance','model-info','depth-info','alignment-info','arm-info'])el(id).textContent='';el('blockers').replaceChildren();}
const mm=p=>Array.isArray(p)?p.map(x=>Number(x).toFixed(1)).join(' / ')+' mm':'不可用';
function result(camera,r){if(!r||r.model!==el('cabinet-model').value)return;if(seen[camera]&&r.image_time<seen[camera])return;seen[camera]=r.image_time;const img=el(camera+'-result');if(typeof r.image==='string'&&r.image.startsWith('data:image/jpeg;base64,')){if(img.getAttribute('src')!==r.image)img.src=r.image;img.hidden=false;}
 el(camera+'-result-status').textContent=`${r.status} · ${new Date(r.image_time*1000).toLocaleTimeString()} · ${r.reason||'冻结帧 '+r.snapshot_id}`;
 if(camera!==activeCamera)return;if(!r.plan||r.status!=='localized'){clearDetails('本轮没有可用入仓路径：'+(r.reason||r.status));return;}const p=r.plan;el('target').textContent=`${camera} · ${r.model==='competition'?'比赛上排中间仓口':'演示仓口'} · 左半区；预插入TCP ${mm(p.tcp_pre_base_mm)}，终点 ${mm(p.tcp_end_base_mm)}`;
 el('tolerance').textContent=`入仓段Z固定 ${p.fixed_z_mm.toFixed(1)} mm，Z变化 ${p.z_range_mm.toFixed(3)} mm；行程 ${p.stroke_mm.toFixed(1)} mm；模型最小净距 ${p.nominal_cad_clearance_mm.toFixed(2)} mm。`;
 el('model-info').textContent=`模型净口 ${p.window_model_size_mm.map(x=>x.toFixed(1)).join(' × ')} mm；实测前沿 ${r.window.measured_size_mm.map(x=>x.toFixed(1)).join(' × ')} mm；深度平面P90残差 ${r.window.rim_plane_p90_mm.toFixed(2)} mm。`;
 el('depth-info').textContent=`CAD内首个障碍深度 ${p.cad_first_obstacle_depth_mm.toFixed(1)} mm，预留后壁 ${p.back_margin_mm.toFixed(1)} mm；电池预计外露 ${p.battery_protrusion_mm.toFixed(1)} mm。`;
 el('alignment-info').textContent=r.alignment?.delta_z_mm!==undefined?`仓外需先对齐：相对当前TCP调整高度约 ${r.alignment.delta_z_mm.toFixed(1)} mm；这不属于固定Z的入仓直线段。`:'仓外对齐尚未完成，不能从当前姿态直接前伸。';
 el('arm-info').textContent=r.arm_path?.status==='kinematic_preview'?`已计算关节路径及立柱检查；TCP插值误差界 ${r.arm_path.interpolation_position_bound_mm.toFixed(3)} mm。${r.arm_path.outside_lift_alignment_required?'候选升降 '+r.arm_path.planning_lift_rad.toFixed(2)+' rad，需先在仓外对齐，双臂升降沿途尚未检查。':''}仍未完成整机与环境碰撞验收。`:`关节路径未通过：${r.arm_path?.reason||'待核对'}；蓝线仅为几何预览。`;
 el('blockers').replaceChildren(...p.blockers.map(text=>{const li=document.createElement('li');li.textContent=text;return li;}));
}
for(const c of ['d435','d405'])el('preview-'+c).onclick=async()=>{activeCamera=c;clearDetails('正在重新识别，旧路径已失效');busy=true;for(const k of ['d435','d405'])el('preview-'+k).disabled=true;el('status').textContent='正在采集配对深度、识别仓口并规划；不发送运动命令…';try{const r=await api('/api/cabinet-preview',{camera:c,model:el('cabinet-model').value});result(c,r);el('status').textContent=r.reason||'识别和水平入仓预览已更新；实机执行保持禁用';}catch(e){el(c+'-result').hidden=true;clearDetails('本轮识别失败：'+e.message);el('status').textContent=e.message;}finally{busy=false;for(const k of ['d435','d405'])el('preview-'+k).disabled=false;}};
el('cabinet-model').onchange=()=>{for(const c of ['d435','d405'])el(c+'-result').hidden=true;activeCamera='d435';for(const c of ['d435','d405'])delete seen[c];clearDetails('模型已切换，请重新识别，不沿用旧落点或路径。');};
async function refresh(){if(busy)return;try{const data=await api('/api/cabinet-preview');for(const c of ['d435','d405'])result(c,data[c]);}catch(e){el('status').textContent='连接中断：'+e.message;}}
async function camera(c){try{const m=await api('/camera-live/'+c+'.json');if(!frameFresh(m,Date.now()))throw Error('stale');const img=el(c+'-live');img.src=cameraImagePath(c,m);img.hidden=false;el(c+'-live-status').textContent='实时彩色；测量和路径来自单独的冻结RGB-D帧';}catch{el(c+'-live').hidden=true;el(c+'-live-status').textContent='相机离线或画面已过期';}}
setInterval(refresh,1500);setInterval(()=>{for(const c of ['d435','d405'])camera(c);},1000);refresh();
