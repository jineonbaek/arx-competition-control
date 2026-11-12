import * as THREE from '../vendor/lift2-preview/three.module.js';
import {OrbitControls} from '../vendor/lift2-preview/OrbitControls.js';
import {frameFresh,cameraImagePath} from './camera-freshness.js';
const el=id=>document.getElementById(id);
let execution=null,revision='',geometry=null,geometryKey='';
const requestId=()=>crypto.randomUUID?.()||([1e7]+-1e3+-4e3+-8e3+-1e11).replace(/[018]/g,c=>(c^crypto.getRandomValues(new Uint8Array(1))[0]&15>>c/4).toString(16));
async function request(path,body){
 const r=await fetch(path,body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:AbortSignal.timeout(10000)}:{cache:'no-store',signal:AbortSignal.timeout(10000)});
 const data=await r.json();if(!r.ok)throw Error(data.message||'请求失败');return data;
}
const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(48,1,.005,5);camera.up.set(0,0,1);
let renderer,orbit;const objects=new THREE.Group();scene.add(objects);
function render(){renderer?.render(scene,camera);}
function clearCloud(){for(const o of [...objects.children]){objects.remove(o);o.geometry.dispose();o.material.dispose();}render();}
try{
 renderer=new THREE.WebGLRenderer({antialias:true});renderer.setClearColor(0x080d16);el('viewport').append(renderer.domElement);
 orbit=new OrbitControls(camera,renderer.domElement);orbit.addEventListener('change',render);
 new ResizeObserver(()=>{const p=el('viewport');renderer.setSize(p.clientWidth,p.clientHeight);camera.aspect=p.clientWidth/Math.max(1,p.clientHeight);camera.updateProjectionMatrix();render();}).observe(el('viewport'));
}catch{el('viewport').textContent='WebGL 不可用；坐标和图像仍可查看。';}
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
function image(id,src){const img=el(id);img.hidden=!src;if(src){if(img.getAttribute('src')!==src)img.src=src;}else img.removeAttribute('src');}
const xyz=p=>p.map(v=>v.toFixed(1)).join(' / ')+' mm';
const toolNames={observe_battery:'电池识别与深度定位',plan_coarse_alignment:'升降与预观察计划',execute_coarse_alignment:'升降与预观察执行',preview_battery_grasp:'FoundationPose 与点云',plan_battery_pick:'抓取路径规划',execute_battery_approach:'张开夹爪与接近',execute_pick:'插入与夹取',verify_hold:'夹持确认'};
function show(data){
 const head=data.observations?.d435,wrist=data.observations?.d405,coarse=data.coarse_alignment,plan=data.pick_plan,g=data.geometry;
 el('run-label').textContent=data.run_id||'等待任务';
 el('status').textContent=data.run_id?'任务 '+data.run_id+' · '+(data.flow?.phase||'starting'):'从下方开始实机流程，工具结果会在这里依次出现。';
 const observation=o=>!o?'等待本轮工具输出':o.status==='localized'?'已定位：'+xyz(o.location.surface_base_mm):(o.message||o.status);
 image('head',head?.recognition?.overlay);image('recognition',wrist?.recognition?.overlay||data.recognition?.overlay);image('pose',data.pose?.overlay);
 el('head-status').textContent=observation(head);el('wrist-status').textContent=observation(wrist);
 el('coarse-status').textContent=coarse?'升降目标 '+coarse.lift.target_rad.toFixed(2)+' rad；预观察腕位 '+xyz(coarse.observer?.target_wrist_base_mm||[]):'等待粗对位计划';
 const fit=data.quality;
 el('quality').textContent=fit?'位姿表面核对：'+(fit.status==='consistent'?'通过':fit.status)+'；中位误差 '+fit.median_abs_depth_mm?.toFixed(1)+' mm，P90 '+fit.p90_abs_depth_mm?.toFixed(1)+' mm；覆盖 '+(fit.mask_coverage*100).toFixed(1)+'%。':'等待本轮 FoundationPose 输出';
 const key=g?data.run_id+g.snapshot_id+JSON.stringify(g.path):'';
 if(key!==geometryKey){geometryKey=key;geometry=g||null;if(g)cloud(g);else clearCloud();}
 el('stamp').textContent=g?'冻结观测 '+g.snapshot_id+' · '+new Date(g.image_time*1000).toLocaleString('zh-CN'):'';
 const rows=[];
 if(g)rows.push(['预抓参考点',xyz(g.path.pre_mm)],['夹持参考点',xyz(g.path.end_mm)]);
 if(plan?.wrist_pre_mm)rows.push(['预抓腕位',xyz(plan.wrist_pre_mm)],['夹持腕位',xyz(plan.wrist_end_mm)],['TCP 标定',plan.calibration_id||'未就绪']);
 el('metrics').replaceChildren(...rows.map(([name,value])=>{const box=document.createElement('div'),dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=name;dd.textContent=value;box.append(dt,dd);return box;}));
 el('blockers').textContent=plan?.calibration_id?'本轮已加载 TCP；抓取路径随新的视觉观测与纠偏更新。':g?'点云已生成，等待抓取计划和 TCP 检查。':'等待任务生成抓取预览';
 el('tool-results').replaceChildren(...(data.tool_results||[]).map(t=>{
  const li=document.createElement('li'),details=document.createElement('details'),summary=document.createElement('summary'),pre=document.createElement('pre');
  summary.textContent=(toolNames[t.name]||t.name)+' · '+(t.ok?'完成':t.error?.message||'等待执行回执');
  pre.textContent=JSON.stringify(t.data||{},null,2);details.append(summary,pre);li.append(details);return li;
 }));
}
function executionButtons(){
 const running=execution?.status==='running';
 el('execute').disabled=running||!execution?.available||!el('environment-clear').checked;
 el('stop-task').disabled=!running;
}
const executionNames={run_created:'建立任务',tool_result:'步骤结果',backend_request:'下发底层请求',execution_review:'路径检查',perception_recovery:'视觉重试',battery_flow:'流程结束',battery_recovery:'控制权处理'};
function executionLog(){
 const list=el('execution-log'),logs=execution?.logs||[];
 if(!logs.length){list.replaceChildren(Object.assign(document.createElement('li'),{textContent:'等待任务启动。'}));return;}
 list.replaceChildren(...logs.map(row=>{
  const data=row.data||{},error=data.error||{},detail=error.code?'['+error.code+'] '+error.message:data.name||data.operation||data.status||(row.event==='battery_recovery'?(data.released_control?'控制权已返还手动控制':'控制权未释放'):data.reason)||data.stage||'';
  return Object.assign(document.createElement('li'),{textContent:new Date(row.time*1000).toLocaleTimeString('zh-CN')+' · '+(executionNames[row.event]||row.event)+(detail?' · '+detail:'')});
 }));list.scrollTop=list.scrollHeight;
}
el('environment-clear').onchange=executionButtons;
el('execute').onclick=async()=>{
 try{
  execution=await request('/api/battery-task',{action:'start',request_id:requestId(),expected_epoch:execution.command_epoch,environment_clear:true});
  revision='';show({run_id:execution.run_id});
  el('execution-status').textContent='自动任务已启动。';
 }catch(e){el('execution-status').textContent=e.message;}executionButtons();
};
el('stop-task').onclick=async()=>{
 try{execution=await request('/api/battery-task',{action:'cancel',request_id:requestId()});el('execution-status').textContent='停止请求已提交。';}
 catch(e){el('execution-status').textContent=e.message;}executionButtons();
};
async function pollExecution(){
 try{
  execution=await request('/api/battery-task');executionButtons();
  const labels={idle:'等待现场确认。',running:'自动任务执行中。',completed:'本轮完成。',needs_attention:'本轮停止，需要检查。',cancelled:'本轮已停止。'};
  el('execution-status').textContent=(labels[execution.status]||execution.status)+(execution.reason?' '+execution.reason:'');
  executionLog();
 }catch(e){execution=null;executionButtons();el('execution-status').textContent=e.message;}
 setTimeout(pollExecution,1000);
}
async function pollResults(){
 try{const data=await request('/api/battery-task/result?after='+encodeURIComponent(revision));if(!data.unchanged){show(data);revision=data.revision||'';}}
 catch(e){el('status').textContent=e.message;}
 setTimeout(pollResults,1000);
}
async function pollCamera(key){
 try{
  const r=await fetch('/camera-live/'+key+'.json',{cache:'no-store',signal:AbortSignal.timeout(2000)});
  if(!r.ok)throw Error('等待相机恢复');const meta=await r.json();
  if(!frameFresh(meta,Date.parse(r.headers.get('Date'))))throw Error('画面离线或过期');
  image(key+'-live',cameraImagePath(key,meta)+'?v='+encodeURIComponent(meta.captured_at));
  el(key+'-live-status').textContent='实时画面 · '+new Date(meta.captured_at*1000).toLocaleTimeString('zh-CN');
 }catch(e){image(key+'-live',null);el(key+'-live-status').textContent=e.message;}
 setTimeout(()=>pollCamera(key),500);
}
pollExecution();pollResults();pollCamera('d435');pollCamera('d405');
