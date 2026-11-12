import * as THREE from '../vendor/lift2-preview/three.module.js';
import {OrbitControls} from '../vendor/lift2-preview/OrbitControls.js';
const el=id=>document.getElementById(id);
let state=null,shown=null,geometry=null,submitting=false,execution=null;
const requestId=()=>crypto.randomUUID?.()||([1e7]+-1e3+-4e3+-8e3+-1e11).replace(/[018]/g,c=>(c^crypto.getRandomValues(new Uint8Array(1))[0]&15>>c/4).toString(16));
const stageNames={thinking:'Agent 读取结果并选择下一阶段',observe_d435:'D435 观察定位',observe_d405:'D405 观察定位',tool:'调用工具',tool_result:'工具返回',recognition:'识别电池并复核分割',foundationpose:'估计电池位姿',pointcloud_path:'核对点云并生成线段',preview_complete:'预览完成',recognition_rejected:'识别未通过',pose_rejected:'位姿与深度表面不一致',handoff_rejected:'两台相机的目标几何未对应'};
const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(48,1,.005,5);camera.up.set(0,0,1);
let renderer,orbit;const objects=new THREE.Group();scene.add(objects);
function render(){renderer?.render(scene,camera);}
function clearCloud(){for(const o of [...objects.children]){objects.remove(o);o.geometry.dispose();o.material.dispose();}render();}
try{
 renderer=new THREE.WebGLRenderer({antialias:true});renderer.setClearColor(0x080d16);el('viewport').append(renderer.domElement);
 orbit=new OrbitControls(camera,renderer.domElement);orbit.addEventListener('change',render);
 new ResizeObserver(()=>{const p=el('viewport');renderer.setSize(p.clientWidth,p.clientHeight);camera.aspect=p.clientWidth/Math.max(1,p.clientHeight);camera.updateProjectionMatrix();render();}).observe(el('viewport'));
}catch{el('viewport').textContent='WebGL 不可用；可查看坐标和图像。';}
function reset(){if(!orbit||!geometry)return;const p=new THREE.Vector3(...geometry.path.end_mm).multiplyScalar(.001);orbit.target.copy(p);camera.position.copy(p).add(new THREE.Vector3(-.3,-.3,.2));orbit.update();render();}
el('reset').onclick=reset;
function cloud(g){
 clearCloud();if(!renderer)return;
 const p=[],c=[];for(const [x,y,z,r,b,h] of g.points){p.push(x,y,z);const color=new THREE.Color((r<<16)|(b<<8)|h);c.push(color.r,color.g,color.b);}
 const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.Float32BufferAttribute(p,3));geo.setAttribute('color',new THREE.Float32BufferAttribute(c,3));
 objects.add(new THREE.Points(geo,new THREE.PointsMaterial({size:.0025,vertexColors:true})));
 const pts=g.path.points_mm.map(p=>new THREE.Vector3(...p).multiplyScalar(.001));
 objects.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),new THREE.LineBasicMaterial({color:0x42e7ec,depthTest:false})));
 [pts[0],pts.at(-1)].forEach((p,i)=>{const m=new THREE.Mesh(new THREE.SphereGeometry(.004),new THREE.MeshBasicMaterial({color:i?0xff9256:0x42e7ec,depthTest:false}));m.position.copy(p);m.renderOrder=2;objects.add(m);});
 reset();
}
function image(id,src){el(id).hidden=!src;if(src)el(id).src=src;else el(id).removeAttribute('src');}
function clear(){image('head',null);el('head-status').textContent='等待头部观察';el('coarse-status').textContent='等待 D435 定位；随后执行固定升降';el('wrist-status').textContent='';geometry=null;clearCloud();image('pose',null);image('recognition',null);el('metrics').replaceChildren();el('quality').textContent='';el('stamp').textContent='';el('blockers').textContent='TCP 未接入：腕部起终点暂不计算，不发送运动指令。';}
const xyz=p=>p.map(v=>v.toFixed(1)).join(' / ')+' mm';
function show(data){
 clear();
 const head=data.observations?.d435,wrist=data.observations?.d405,coarse=data.coarse_alignment;
 const observationStatus=o=>!o?'尚未进行':o.status==='localized'?'已定位：可见表面 '+xyz(o.location.surface_base_mm):o.status==='target_not_found'?'本帧未找到电池':o.status==='depth_rejected'?'识别到电池，但深度不足：'+o.message:'识别或分割复核未通过';
 image('head',head?.recognition?.overlay);el('head-status').textContent=observationStatus(head);
 el('wrist-status').textContent=observationStatus(wrist);
 if(coarse)el('coarse-status').textContent='固定升降 '+coarse.lift.target_rad.toFixed(3)+' SDK rad；右臂到固定观察位 '+xyz(coarse.observer?.target_wrist_base_mm||[])+'，再由 D405 自适应抓取。'+(coarse.lift.column_sweep?.status==='column_clear'?coarse.lift.column_sweep.message+'。':'')+coarse.blockers.join('；')+'。未执行运动。';
 image('recognition',data.recognition?.overlay||wrist?.recognition?.overlay);image('pose',data.pose?.overlay);
 el('status').textContent=data.message;
 const fit=data.quality;
 let fitText=fit?'CAD 深度表面核对：'+({consistent:'一致',rejected:'需复核',insufficient_depth:'有效深度不足'}[fit.status]||fit.status)+'；中位偏差 '+(fit.median_abs_depth_mm?.toFixed(1)??'无')+' mm，P90 '+(fit.p90_abs_depth_mm?.toFixed(1)??'无')+' mm；轮廓覆盖 '+(100*fit.mask_coverage).toFixed(1)+'%。仅为一致性筛查，尚未验证实物精度。':'';
 const handoff=data.camera_handoff;
 if(handoff){const h=handoff.quality;fitText+=' D435→D405 接管核对：'+(h?.status==='consistent'?'几何一致':'未通过')+'；头部深度中位偏差 '+(h?.median_abs_depth_mm?.toFixed(1)??'无')+' mm，P90 '+(h?.p90_abs_depth_mm?.toFixed(1)??'无')+' mm。'+handoff.note;}
 el('quality').textContent=fitText;
 if(!data.geometry)return;
 geometry=data.geometry;cloud(geometry);
 const g=geometry;
 const rows=[['预抓参考点 G',xyz(g.path.pre_mm)],['夹持参考点 G',xyz(g.path.end_mm)],['CAD 夹指伸入量',g.grip.insertion_mm+' mm'],['腕部起终点','等待 TCP 标定']];
 el('metrics').replaceChildren(...rows.map(([name,value])=>{const box=document.createElement('div'),dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=name;dd.textContent=value;box.append(dt,dd);return box;}));
 el('stamp').textContent='冻结快照 '+g.snapshot_id+' · '+new Date(g.image_time*1000).toLocaleString('zh-CN');
 el('quality').textContent=fitText+' 电池有效深度 '+g.depth.valid_pixels+' / '+g.depth.mask_pixels+'（'+(g.depth.valid_ratio*100).toFixed(1)+'%）；路径已跟随电池插入轴，相对整车 +X '+g.grip.approach_x_error_deg.toFixed(1)+'°；凸起朝下偏差 '+g.grip.bump_down_error_deg.toFixed(1)+'°。这些数值供核对，不代表碰撞检查通过。';
 el('blockers').textContent=g.blockers.join('；')+'。'+g.note;
}
async function request(path,body){
 const r=await fetch(path,body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:AbortSignal.timeout(10000)}:{cache:'no-store',signal:AbortSignal.timeout(10000)});
 const data=await r.json();if(!r.ok)throw Error(data.message||'请求失败');return data;
}
function buttons(){el('start').disabled=submitting||!state||!state.available||state.busy||!el('distance').validity.valid||!el('distance').value;el('cancel').disabled=!state?.busy;el('distance').disabled=!!state?.busy;}
el('distance').oninput=buttons;
el('start').onclick=async()=>{
 try{
  submitting=true;clear();shown=null;buttons();
  // Each preview owns a fresh Agent run; old receipts remain on disk.
  state=await request('/api/grasp-integration',{action:'new',conversation_id:state.conversation_id});
  state=await request('/api/grasp-integration',{action:'send',conversation_id:state.conversation_id,request_id:requestId(),message:'按抓取计划从 D435 观察定位开始，规划升降粗对位，再检查当前 D405 视野并生成 FoundationPose、点云和水平路径预览。预抓距离 '+el('distance').value+' mm。TCP 留空；本次只预览，不移动任何硬件。'});
 }catch(e){el('status').textContent=e.message;}finally{submitting=false;buttons();}
};
el('cancel').onclick=async()=>{try{state=await request('/api/grasp-integration',{action:'cancel',conversation_id:state.conversation_id,request_id:state.job.id});}catch(e){el('status').textContent=e.message;}buttons();};
async function poll(){
 try{
  state=await request('/api/grasp-integration');buttons();
  el('stages').textContent=state.busy?(state.job.status==='cancelling'?'正在终止计算并释放模型':stageNames[state.job.phase]||state.job.phase||'处理中'):'';
  if(state.busy)el('status').textContent='本次只计算预览，不控制机器人。';
  else if(state.job&&shown!==state.job.id){
   shown=state.job.id;
   el('status').textContent=state.job.error||state.messages.at(-1)?.content||'计算已取消';
   if(state.job.status==='done'){
    try{show(await request('/api/grasp-integration/result'));}catch(e){if(!state.messages.length)el('status').textContent=e.message;}
   }
  }else if(!state.job)el('status').textContent=state.available?'准备就绪。Agent 将从 D435 开始，按需采集并运行模型。':state.unavailable_reason;
 }catch(e){el('status').textContent=e.message;}
 setTimeout(poll,1000);
}
poll();

function executionButtons(){
 const running=execution?.status==='running';
 el('execute').disabled=running||!execution?.available||!el('environment-clear').checked;
 el('stop-task').disabled=!running;
}
const executionNames={run_created:'建立任务',tool_result:'步骤结果',backend_request:'下发底层请求',execution_review:'路径检查',perception_recovery:'视觉重试',battery_flow:'流程结束'};
function executionLog(){
 const list=el('execution-log'),logs=execution?.logs||[];
 if(!logs.length){list.replaceChildren(Object.assign(document.createElement('li'),{textContent:'等待任务启动。'}));return;}
 list.replaceChildren(...logs.map(row=>{
  const data=row.data||{},error=data.error||{},detail=error.code?'['+error.code+'] '+error.message:data.name||data.operation||data.status||data.reason||data.stage||'';
  return Object.assign(document.createElement('li'),{textContent:new Date(row.time*1000).toLocaleTimeString('zh-CN')+' · '+(executionNames[row.event]||row.event)+(detail?' · '+detail:'')});
 }));
 list.scrollTop=list.scrollHeight;
}
el('environment-clear').onchange=executionButtons;
el('execute').onclick=async()=>{
 try{
  execution=await request('/api/battery-task',{action:'start',request_id:requestId(),expected_epoch:execution.command_epoch,environment_clear:true});
  el('execution-status').textContent='自动任务已启动；停止按钮始终优先。';
 }catch(e){el('execution-status').textContent=e.message;}executionButtons();
};
el('stop-task').onclick=async()=>{
 try{execution=await request('/api/battery-task',{action:'cancel',request_id:requestId()});el('execution-status').textContent='停止请求已提交。';}
 catch(e){el('execution-status').textContent=e.message;}executionButtons();
};
async function pollExecution(){
 try{
  execution=await request('/api/battery-task');executionButtons();
  const labels={idle:'执行后端就绪，等待现场确认。',running:'自动任务执行中。',completed:'本轮完成。',needs_attention:'本轮停止，需要检查。',awaiting_hold_verification:'夹爪反馈尚未确认夹持。',owned_by_other:'自动任务由另一个网页会话占用。'};
  el('execution-status').textContent=(labels[execution.status]||execution.status)+(execution.reason?' '+execution.reason:'');
  executionLog();
 }catch(e){execution=null;executionButtons();el('execution-status').textContent=e.message;}
 setTimeout(pollExecution,1000);
}
pollExecution();
