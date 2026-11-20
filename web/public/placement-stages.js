'use strict';
(()=>{
  const $=id=>document.getElementById(id);
  if(!$('stage-status'))return;
  let latest=null,sending=false,initialized=false,pointMode=null,feedbackRequest=null;
  const fields={cabinet_z_mm:'stage-cabinet-z',target_z_mm:'stage-target-z',correction_step_mm:'stage-step',
    tolerance_mm:'stage-tolerance',y_target_offset_mm:'stage-y-offset',insert_distance_mm:'stage-insert-distance',insert_speed_mm_s:'stage-insert-speed'};
  const sourceNames={d405_fixed_cabinet:'D405 固定仓口参照',manual_translation_calibration:'手动高度偏置（不含倾斜测量）',model_uncompensated:'模型高度（未补偿底盘误差）',unavailable:'尚无有效离地高度'};
  const phaseNames={clamp:'保持右夹爪闭合',preobserve:'进入预观测点',correct:'D405 修正',d405_observe:'D405 深度观测',d405_correct:'D405 位移',insert:'入仓',z:'单独调整 Z',hold:'保持',idle:'待命',measure:'测量仓外间距',push:'推入',verify:'复查入仓深度',compensate:'补偿推入',verified:'入仓已确认',unverified:'入仓未确认',release:'松爪',exit:'原路退出'};
  const statusNames={idle:'待命',running:'进行中',completed:'完成并保持',completed_travel:'推进行程完成',completed_verified:'入仓已确认，已松爪退出',completed_unverified:'入仓未确认，已松爪退出',insertion_unverified:'入仓未确认',blocked:'本阶段未执行',held:'已保持'};
  const number=(id)=>{const text=String($(id).value).trim(),value=Number(text);if(!text||!Number.isFinite(value))throw Error('请填写有效数字：'+id);return value;};
  const mm=value=>typeof value==='number'&&Number.isFinite(value)?value.toFixed(1)+' mm':'—';
  const seconds=value=>typeof value==='number'&&Number.isFinite(value)?value.toFixed(1)+' s':'—';
  const jointIds=Array.from({length:6},(_,i)=>'stage-joint-'+(i+1));
  const xyzIds=['stage-xyz-x','stage-xyz-y','stage-xyz-z'],rpyIds=['stage-rpy-r','stage-rpy-p','stage-rpy-y'];
  function fillPoint(j){
    jointIds.forEach((id,i)=>$(id).value=Number(j.settings.preobserve_joints_deg[i]).toFixed(3));
    xyzIds.forEach((id,i)=>$(id).value=Number(j.preobserve_tcp_mm?.[i]||0).toFixed(2));
    rpyIds.forEach((id,i)=>$(id).value=Number(j.preobserve_tool_rpy_deg?.[i]||0).toFixed(2));
    $('stage-point-source').textContent=j.settings.preobserve_source+' · XYZ 为当前升降读数下的机器人坐标';pointMode=null;
  }
  function fill(j){
    if(!j.settings)return;
    for(const [key,id] of Object.entries(fields))$(id).value=String(j.settings[key]);
    $('stage-provisional').checked=j.settings.cabinet_height_provisional;
    $('stage-z-reference').value=j.settings.z_reference;$('stage-correct-axes').value=j.settings.correction_axes;
    fillPoint(j);initialized=true;
  }
  for(const id of [...xyzIds,...rpyIds])$(id).oninput=()=>{pointMode='cartesian';$('stage-save-status').textContent='TCP 点位已编辑，保存或执行预观测时解算';};
  for(const id of jointIds)$(id).oninput=()=>{pointMode='joint';$('stage-save-status').textContent='关节点已编辑，保存后生效';};
  for(const id of [...Object.values(fields),'stage-z-reference','stage-correct-axes','stage-provisional'])$(id).oninput=()=>{$('stage-save-status').textContent='参数已编辑，保存或执行时生效';if(id==='stage-z-reference')buttons();};
  function buttons(){
    const ready=!!latest?.can_start&&!sending;
    for(const id of ['stage-preobserve','stage-correct','stage-insert','stage-record-point','stage-save','stage-calibrate-z'])$(id).disabled=!ready;
    const chosen=$('stage-z-reference').value;
    const hasHeight=chosen==='model'?latest?.height?.model_tcp_z_mm!=null:chosen==='manual'?latest?.manual_anchor_valid:latest?.height?.source==='d405_fixed_cabinet';
    for(const id of ['stage-z-minus-5','stage-z-minus-1','stage-z-plus-1','stage-z-plus-5','stage-z-target'])$(id).disabled=!ready||!hasHeight;
    $('stage-stop').disabled=!(latest?.running&&latest?.request_id)||sending;
    $('stage-observe').disabled=sending;
    $('stage-observe').textContent=latest?.observing?'停止 D405 实时观测':'开始 D405 实时观测（不运动）';
  }
  function render(j){
    latest=j;if(!initialized)fill(j);
    if(feedbackRequest && feedbackRequest===j.request_id)showRunFeedback(j);
    const h=j.height||{},selected=j.selected_height||{};
    $('stage-z').textContent=mm(h.tcp_z_mm);$('stage-model-z').textContent=mm(h.model_tcp_z_mm);
    $('stage-relative-z').textContent=mm(h.relative_cabinet_z_mm);
    $('stage-height-source').textContent=(sourceNames[h.source]||h.source||'等待反馈')+(typeof h.age_s==='number'?` · ${(h.age_s).toFixed(1)} 秒前`: '')+(j.settings?.cabinet_height_provisional?' · 仓口高度暂定':'');
    $('stage-height-offset').textContent='综合 Z 修正：'+mm(h.visual_z_correction_mm)+'（含底盘、升降及标定误差，不能单独归因于底盘）';
    $('stage-lift').textContent=`升降 ${typeof j.lift_rad==='number'?j.lift_rad.toFixed(3):'—'} rad · 模型贡献 ${mm(j.lift_model_contribution_mm)}`;
    const basisReason=!j.geometry&&j.observation_error?j.observation_error:selected.reason;
    $('stage-basis').textContent='Z 动作基准：'+(sourceNames[selected.source]||'不可用')+' · '+mm(selected.tcp_z_mm)+(selected.tcp_z_mm!=null?' · 距目标 '+mm(j.settings.target_z_mm-selected.tcp_z_mm):'')+(basisReason?' · '+basisReason:'');
    $('stage-status').textContent=stageDescription(j)+(j.config_error?' · '+j.config_error:'')+(j.recording_error?' · 记录文件保存失败：'+j.recording_error:'');
    $('stage-observation-status').textContent=(j.observing?'实时观测已开启':'实时观测未开启')+(j.observation_error?' · '+j.observation_error:j.geometry?` · 深度支持 ${j.geometry.depth_support_pixels} 像素；仓口高度 ${mm(j.geometry.opening_height_mm)}`:'');
    const push=j.push||{};
    $('stage-travel').textContent='实际推进：'+mm(j.measured_travel_mm??push.measured_travel_mm);
    const verify=push.insertion_verification||push;
    $('stage-insertion-evidence').textContent='仓外间距 '+mm(verify.initial_gap_mm)+'；计划推进 '+mm(verify.planned_initial_travel_mm)+'；实测入仓深度 '+mm(verify.verified_depth_mm)+'；补偿 '+(verify.compensation_count??0)+' 次 / '+mm(verify.compensation_mm)+(typeof verify.insertion_depth_verified==='boolean'?'；'+(verify.insertion_depth_verified?'已确认入仓':'尚未确认入仓'):'')+(push.reason?'；'+push.reason:'');
    if(j.running&&j.operation==='insert')$('stage-status').textContent='入仓 · '+(phaseNames[push.phase]||push.phase||'正在启动')+(push.reason?' · '+push.reason:'');
    const rows=[...(j.timeline||[])];if(j.active_phase)rows.push(j.active_phase);
    $('stage-timings').replaceChildren();
    if(!rows.length){const tr=document.createElement('tr'),td=document.createElement('td');td.colSpan=5;td.textContent='等待操作';tr.append(td);$('stage-timings').append(tr);}
    for(const row of rows){const tr=document.createElement('tr');for(const text of [phaseNames[row.phase]||row.phase,seconds(row.planned_duration_s),seconds(row.duration_s),seconds(row.cumulative_duration_s),row.status||'']){const td=document.createElement('td');td.textContent=text;tr.append(td);}$('stage-timings').append(tr);}
    buttons();
  }
  async function post(data){
    const r=await fetch('/api/placement-stages',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});const j=await r.json();
    if(!r.ok)throw Error(j.message||'请求未确认');render(j);return j;
  }
  function settings(){
    if(!latest?.settings)throw Error('参数尚未读取');
    const value={...latest.settings,preobserve_joints_deg:jointIds.map(number),cabinet_height_provisional:$('stage-provisional').checked,
      z_reference:$('stage-z-reference').value,correction_axes:$('stage-correct-axes').value};
    for(const [key,id] of Object.entries(fields))value[key]=number(id);
    if(pointMode==='joint')value.preobserve_source='网页关节点编辑';return value;
  }
  async function save(){
    if(pointMode==='cartesian'){const j=await post({op:'save_cartesian',xyz_mm:xyzIds.map(number),rpy_deg:rpyIds.map(number)});fillPoint(j);}
    const j=await post({op:'save_settings',settings:settings()});fill(j);$('stage-save-status').textContent='已保存到车载本地；保存没有发送运动';return j;
  }
  function stageDescription(j){
    const alreadyThere=j?.operation==='preobserve'&&j.status==='completed'&&j.timeline?.some(row=>row.phase==='preobserve'&&row.planned_duration_s===0)&&!j.trajectory_handoffs?.length;
    return (phaseNames[j?.operation]||'独立控制')+' · '+(alreadyThere?'当前位置已在预观测点，无需移动；可修改点位或选择下一阶段':j?.reason||statusNames[j?.status]||j?.status||'待命');
  }
  function showRunFeedback(j){
    const feedback=$('stage-feedback');
    const pending=j.status==='running';
    const successful=['completed','completed_travel','completed_verified'].includes(j.status);
    const time=j.active_phase?.planned_duration_s;
    const message=stageDescription(j)+(pending && typeof time==='number'?' · 计划 '+seconds(time):'');
    if(feedback.textContent!==message)feedback.textContent=message;
    feedback.dataset.level=pending?'pending':successful?'success':'error';
    if(!pending)feedbackRequest=null;
  }
  async function action(call,label='操作'){
    if(sending)return;sending=true;feedbackRequest=null;buttons();
    const feedback=$('stage-feedback');feedback.textContent=label+' · 正在提交，请稍候…';feedback.dataset.level='pending';
    try{
      const result=await call();
      if(result?.request_id && feedbackRequest===result.request_id)showRunFeedback(result);
      else{feedback.textContent=label+' · 已收到回执 · '+stageDescription(latest);feedback.dataset.level='success';}
    }
    catch(e){feedback.textContent=label+' · 未收到成功回执：'+e.message+'；请查看上方当前状态';feedback.dataset.level='error';}
    finally{sending=false;buttons();}
  }
  async function run(phase,delta){
    const j=await save();const data={op:'run',phase,request_id:crypto.randomUUID?.()||'stage-'+Date.now()+'-'+Math.random().toString(16).slice(2),expected_epoch:j.command_epoch};
    if(delta!==undefined)data.delta_z_mm=delta;
    const result=await post(data);feedbackRequest=data.request_id;return result;
  }
  $('stage-save').onclick=()=>action(save,'保存参数与预观测点');
  $('stage-record-point').onclick=()=>action(async()=>{const j=await post({op:'record_preobserve'});fillPoint(j);$('stage-save-status').textContent='已记录当前位置为预观测点，没有发送运动';});
  for(const [id,phase] of [['stage-preobserve','preobserve'],['stage-correct','correct'],['stage-insert','insert']])$(id).onclick=()=>action(()=>run(phase),phaseNames[phase]);
  for(const [id,delta] of [['stage-z-minus-5',-5],['stage-z-minus-1',-1],['stage-z-plus-1',1],['stage-z-plus-5',5]])$(id).onclick=()=>action(()=>run('z',delta));
  $('stage-z-target').onclick=()=>action(()=>run('z'));
  $('stage-stop').onclick=()=>action(async()=>{if(latest?.request_id&&latest.running)await post({op:'cancel',request_id:latest.request_id});});
  $('stage-observe').onclick=()=>action(()=>post({op:latest?.observing?'observe_off':'observe_on'}));
  $('stage-calibrate-z').onclick=()=>action(async()=>{await post({op:'calibrate_z',known_tcp_z_mm:number('stage-known-z')});$('stage-z-reference').value='manual';await save();});
  async function poll(){
    try{const r=await fetch('/api/placement-stages',{cache:'no-store'});const j=await r.json();if(!r.ok)throw Error(j.message||'读取失败');render(j);}
    catch(e){latest=null;buttons();$('stage-status').textContent='独立控制连接中断：'+e.message;}
    setTimeout(poll,400);
  }
  poll();
})();
