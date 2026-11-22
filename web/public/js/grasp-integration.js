import * as THREE from '../vendor/lift2-preview/three.module.js';
import {OrbitControls} from '../vendor/lift2-preview/OrbitControls.js';
import {frameFresh,cameraImagePath} from './camera-freshness.js';
const el=id=>document.getElementById(id);
let execution=null,revision='',geometry=null,geometryKey='',settingsLoaded=false;
let recognitionProfiles=[],referenceCatalog=[];
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
const joints=q=>q?.length===6?q.map(v=>(v*180/Math.PI).toFixed(1)+'°').join(' / '):'尚未记录';
function showPlanC(profile={}){
 el('plan-c-pregrasp-status').textContent=joints(profile.pregrasp_joints_rad);
 el('plan-c-grasp-status').textContent=joints(profile.grasp_joints_rad);
}
const selectedRecognitionProfile=()=>recognitionProfiles.find(p=>p.id===el('recognition-profile').value);
function recognitionProfileFromForm(identity=el('recognition-profile').value){
 const refs={first:[...el('reference-first').selectedOptions].map(o=>o.value),reobserve:[...el('reference-reobserve').selectedOptions].map(o=>o.value)};
 return {id:identity,name:el('recognition-profile-name').value.trim(),dino_prompt:el('dino-prompt').value.trim(),qwen_task:el('qwen-task').value.trim(),...(refs.first.length||refs.reobserve.length?{reference_images:refs}:{})};
}
function showReferences(refs={}){
 for(const stage of ['first','reobserve']){
  const select=el('reference-'+stage),chosen=refs[stage]||[];
  select.replaceChildren(...referenceCatalog.map(row=>Object.assign(document.createElement('option'),{value:row.id,textContent:row.label,selected:chosen.includes(row.id)})));
  referencePreview(stage);
 }
}
function referencePreview(stage){
 const selected=[...el('reference-'+stage).selectedOptions].map(o=>o.value);
 el('reference-'+stage+'-preview').replaceChildren(...referenceCatalog.filter(row=>selected.includes(row.id)).map(row=>{
  const img=document.createElement('img');img.src=row.url;img.alt=row.label;img.title=row.label;img.style.width='140px';return img;
 }));
}
for(const stage of ['first','reobserve'])el('reference-'+stage).onchange=()=>referencePreview(stage);
function showRecognitionProfile(profile){
 if(!profile)return;
 el('recognition-profile').value=profile.id;el('recognition-profile-name').value=profile.name;
 el('dino-prompt').value=profile.dino_prompt;el('qwen-task').value=profile.qwen_task;
 showReferences(profile.reference_images);
 el('delete-recognition-profile').disabled=!profile.id.startsWith('custom-');
 el('recognition-profile-status').textContent='当前加载：'+profile.name;
}
function showRecognitionProfiles(settings={},preferred){
 recognitionProfiles=settings.recognition_profiles||recognitionProfiles;
 referenceCatalog=settings.reference_catalog||referenceCatalog;
 const select=el('recognition-profile'),identity=preferred||settings.selected_recognition_profile_id||select.value||'handle';
 select.replaceChildren(...recognitionProfiles.map(profile=>Object.assign(document.createElement('option'),{value:profile.id,textContent:profile.name})));
 showRecognitionProfile(recognitionProfiles.find(profile=>profile.id===identity)||recognitionProfiles[0]);
}
async function manageRecognitionProfile(action,profile){
 try{
  execution=await request('/api/battery-task',{action,request_id:requestId(),...(profile?{profile}:{profile_id:el('recognition-profile').value})});
  showRecognitionProfiles(execution.settings,profile?.id);el('recognition-profile-status').textContent='已保存并加载：'+el('recognition-profile-name').value;
 }catch(e){el('recognition-profile-status').textContent=e.message;}executionButtons();
}
function saveRecognitionProfile(forceNew){
 for(const id of ['recognition-profile-name','dino-prompt','qwen-task'])if(!el(id).reportValidity())return;
 const selected=selectedRecognitionProfile();let identity=selected?.id;
 if(forceNew||!identity?.startsWith('custom-'))identity='custom-'+requestId();
 manageRecognitionProfile('save_recognition_profile',recognitionProfileFromForm(identity));
}
const toolNames={observe_battery:'电池识别与深度定位',plan_coarse_alignment:'升降与预观察计划',execute_coarse_alignment:'升降与预观察执行',preview_battery_grasp:'FoundationPose 与点云',plan_battery_pick:'抓取路径规划',execute_battery_approach:'张开夹爪与接近',execute_pick:'插入与夹取',verify_hold:'夹持确认',stow_for_carry:'底盘后退 200 mm 并停车',return_to_carry:'返回运输姿态'};
function show(data){
 const head=data.observations?.head||data.observations?.d435,wrist=data.observations?.d405,coarse=data.coarse_alignment,plan=data.pick_plan,g=data.geometry;
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
 if(g)rows.push(['TCP 预抓点',xyz(g.path.pre_mm)],['TCP 终点',xyz(g.path.end_mm)]);
 if(plan?.wrist_pre_mm)rows.push(['预抓腕位',xyz(plan.wrist_pre_mm)],['夹持腕位',xyz(plan.wrist_end_mm)],['TCP 标定',plan.calibration_id||'未就绪']);
 el('metrics').replaceChildren(...rows.map(([name,value])=>{const box=document.createElement('div'),dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=name;dd.textContent=value;box.append(dt,dd);return box;}));
 el('blockers').textContent=plan?.calibration_id?'本轮已加载 TCP；抓取路径随新的视觉观测与纠偏更新。':g?'点云已生成，等待抓取计划和 TCP 检查。':'等待任务生成抓取预览';
 el('tool-results').replaceChildren(...(data.tool_results||[]).map(t=>{
  const li=document.createElement('li'),details=document.createElement('details'),summary=document.createElement('summary'),pre=document.createElement('pre');
  summary.textContent=(toolNames[t.name]||t.name)+' · '+(t.ok?'完成':t.error?.message||'等待执行回执');
  pre.textContent=JSON.stringify(t.data||{},null,2);details.append(summary,pre);li.append(details);return li;
 }));
}
let actionBusy=false;
function executionButtons(){
 const running=execution?.status==='running';
 if(running){
  const activeFast=execution.settings?.pickup_flow==='fast';
  el(activeFast?'flow-fast':'flow-stable').checked=true;
 }
 const fast=el('flow-fast').checked,mode=el('vision-mode').value;
 const planD=!fast&&mode==='plan_d',planF=mode==='plan_f';
 const start=execution?.settings?.fast_start,policy=execution?.settings?.fast_dock_policy;
 el('flow-switch').disabled=running||actionBusy||planDBusy;
 el('stable-settings').hidden=fast;
 el('fast-settings').hidden=!fast;
 el('plan-d-panel').hidden=!planD;
 el('execute').textContent=fast?'启动快模式':'启动稳模式';
 el('execute').disabled=actionBusy||planDBusy||running||!execution?.available||
  !el('environment-clear').checked||(fast?(!start||!policy):planD);
 const paths={plan_f:'D405 粗定位、精定位后夹取',d405:'D435 粗定位、D405 精定位后夹取',
  d435:'D435 单帧定位后夹取',fixed:'按预录关节位置夹取'};
 el('flow-summary').textContent=fast?
   'T1 低速贴靠 0.2 秒 → 雷达确认停稳 → 固定预抓取点 → D405 精调 → 夹取 → 后退 200 mm。'+
  (execution&&!policy?' 等待配置平台停车信号。':''):
  planD?'Plan D 仅预览 D435 电池识别结果。':(paths[mode]||'按所选方案夹取')+' → 后退 200 mm → 返回运输姿态。';
 el('fast-reference').textContent=start?'地图出发点  X '+start.pose[0].toFixed(3)+' m  Y '+start.pose[1].toFixed(3)+' m  朝向 '+(start.pose[2]*180/Math.PI).toFixed(2)+'°':'尚未记录快流程出发点';
 el('fast-record').disabled=!execution?.available||actionBusy;
 el('plan-d-capture').disabled=running||planDBusy||actionBusy;
 el('stop-task').disabled=!running;
 el('vision-mode').disabled=running;
 el('lift-target-rad').disabled=running;el('observer-z-mm').disabled=running||planF;
 el('plan-c-lift-target-rad').disabled=running;
 el('record-plan-c-pregrasp').disabled=running;el('record-plan-c-grasp').disabled=running;
 for(const id of ['recognition-profile','recognition-profile-name','dino-prompt','qwen-task','reference-first','reference-reobserve','save-recognition-profile','save-as-recognition-profile'])el(id).disabled=running;
 el('delete-recognition-profile').disabled=running||!el('recognition-profile').value.startsWith('custom-');
}
const executionNames={grasp_point2:'记录抓取起始姿态',grasp_pose_retry:'预观察点 3 重试',grasp_arrival:'到位误差记录',fast_dock:'快流程贴靠',fast_pregrasp:'快模式预抓取点',run_created:'建立任务',tool_result:'步骤结果',backend_request:'下发底层请求',execution_review:'路径检查',perception_recovery:'视觉重试',battery_flow:'流程结束',battery_recovery:'控制权处理'};
function executionLog(){
 const list=el('execution-log'),logs=execution?.logs||[];
 if(!logs.length){list.replaceChildren(Object.assign(document.createElement('li'),{textContent:'等待任务启动。'}));return;}
 list.replaceChildren(...logs.map(row=>{
  const data=row.data||{},error=data.error||{},detail=error.code?'['+error.code+'] '+error.message:data.name||data.operation||data.status||(row.event==='battery_recovery'?(data.released_control?'控制权已返还手动控制':'控制权未释放'):data.reason)||data.stage||'';
  return Object.assign(document.createElement('li'),{textContent:new Date(row.time*1000).toLocaleTimeString('zh-CN')+' · '+(executionNames[row.event]||row.event)+(detail?' · '+detail:'')});
 }));list.scrollTop=list.scrollHeight;
}
el('environment-clear').onchange=executionButtons;
el('vision-mode').onchange=executionButtons;
el('recognition-profile').onchange=async()=>{
 const profile=selectedRecognitionProfile();showRecognitionProfile(profile);
 if(profile)await manageRecognitionProfile('select_recognition_profile');
};
el('save-recognition-profile').onclick=()=>saveRecognitionProfile(false);
el('save-as-recognition-profile').onclick=()=>saveRecognitionProfile(true);
el('delete-recognition-profile').onclick=()=>manageRecognitionProfile('delete_recognition_profile');
try{
 if(sessionStorage.getItem('pickup-flow')==='fast')el('flow-fast').checked=true;
}catch{}
el('flow-switch').onchange=()=>{
 try{sessionStorage.setItem('pickup-flow',el('flow-fast').checked?'fast':'stable');}catch{}
 executionButtons();
};
el('fast-record').onclick=async()=>{
 if(actionBusy||el('fast-record').disabled)return;
 actionBusy=true;executionButtons();
 try{
  execution=await request('/api/battery-task',{action:'record_fast_start',request_id:requestId()});
  el('execution-status').textContent='快流程出发点已保存。';
 }catch(e){el('execution-status').textContent=e.message;}
 finally{actionBusy=false;executionButtons();}
};
el('execute').onclick=async()=>{
 if(actionBusy||el('execute').disabled)return;
 const fast=el('flow-fast').checked;
 actionBusy=true;executionButtons();
 try{
  const payload={action:fast?'start_fast':'start',request_id:requestId(),expected_epoch:execution.command_epoch,environment_clear:true};
  if(!fast){
   const lift=el('lift-target-rad'),observer=el('observer-z-mm'),planCLift=el('plan-c-lift-target-rad');
   const mode=el('vision-mode').value;
   const invalid=[lift,observer,planCLift,...['recognition-profile-name','dino-prompt','qwen-task'].map(el)].find(field=>!field.checkValidity());
   if(invalid){el('stable-settings').open=true;invalid.closest('details').open=true;invalid.reportValidity();return;}
   Object.assign(payload,{lift_target_rad:lift.valueAsNumber,observer_z_mm:observer.valueAsNumber,vision_mode:mode,recognition_profile:recognitionProfileFromForm()});
   if(mode==='fixed')payload.plan_c_lift_target_rad=planCLift.valueAsNumber;
  }
  execution=await request('/api/battery-task',payload);
  revision='';show({run_id:execution.run_id});
  el('execution-status').textContent=fast?'快模式已启动。':'稳模式已启动。';
 }catch(e){el('execution-status').textContent=e.message;}
 finally{actionBusy=false;executionButtons();}
};
async function recordPlanC(point){
 try{
  const lift=el('plan-c-lift-target-rad');if(!lift.reportValidity())return;
  execution=await request('/api/battery-task',{action:'record_plan_c',request_id:requestId(),point,lift_target_rad:lift.valueAsNumber});
  showPlanC(execution.settings?.plan_c);el('execution-status').textContent=(point==='pregrasp'?'预夹取点':'夹取点')+'已保存。';
 }catch(e){el('execution-status').textContent=e.message;}executionButtons();
}
el('record-plan-c-pregrasp').onclick=()=>recordPlanC('pregrasp');
el('record-plan-c-grasp').onclick=()=>recordPlanC('grasp');
el('stop-task').onclick=async()=>{
 try{execution=await request('/api/battery-task',{action:'cancel',request_id:requestId()});el('execution-status').textContent='停止请求已提交。';}
 catch(e){el('execution-status').textContent=e.message;}executionButtons();
};
async function pollExecution(){
 try{
  execution=await request('/api/battery-task');
  if(!settingsLoaded&&execution.settings?.lift_target_rad!=null&&execution.settings?.observer_z_mm!=null){
   el('lift-target-rad').value=execution.settings.lift_target_rad;el('observer-z-mm').value=execution.settings.observer_z_mm;el('vision-mode').value=execution.settings.vision_mode||'d405';
   if(execution.settings.plan_c?.lift_target_rad!=null)el('plan-c-lift-target-rad').value=execution.settings.plan_c.lift_target_rad;
    showPlanC(execution.settings.plan_c);showRecognitionProfiles(execution.settings,execution.settings.recognition_profile?.id);settingsLoaded=true;
  }
  executionButtons();
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
let planDBusy=false;
function showPlanD(data){
 el('plan-d-status').textContent=data.reason||data.status;
 image('plan-d-image',data.image||null);
 const m=data.metrics;
 el('plan-d-metrics').textContent=m?'深度 '+(m.depth_m*1000).toFixed(1)+' mm · 置信度 '+(m.confidence*100).toFixed(1)+'% · '+new Date(data.captured_at*1000).toLocaleTimeString('zh-CN'):'';
}
el('plan-d-capture').onclick=async()=>{
 if(planDBusy||execution?.status==='running')return;
 planDBusy=true;executionButtons();image('plan-d-image',null);el('plan-d-metrics').textContent='';
 try{
  let data=await request('/api/plan-d-vision',{op:'capture'});showPlanD(data);
  const deadline=Date.now()+25000;
  while(data.running&&Date.now()<deadline){
   await new Promise(resolve=>setTimeout(resolve,350));
   data=await request('/api/plan-d-vision');showPlanD(data);
  }
  if(data.running)throw Error('采集仍在后台进行，请稍后重新读取');
 }catch(e){el('plan-d-status').textContent=e.message;image('plan-d-image',null);}
 finally{planDBusy=false;executionButtons();}
};
pollExecution();pollResults();pollCamera('d435');pollCamera('d405');
