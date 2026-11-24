'use strict';
const $=id=>document.getElementById(id);
const labels={seat_push:"连续推向仓底",approach_mouth:'沿入口轴到仓口',fixed_push:'固定朝向推入 150 mm',idle:'待命',entry:'准备当前任务',clamp:'自动夹紧电池',visual_chassis:'D435 修正底盘',chassis_lock:'底盘停止并锁定',fixed_entry:'执行预录关节位置',d405_preobservation:'到达 D405 预观察位',d405_horizontal_trim:'D405 左右微调',full_insert:'按仓深和实测间距推入',supplemental_insert:'补偿推入',measure:'测量仓外间距',verify:'校验入仓深度',compensate:'补偿推入',unverified:'入仓未确认',verified:'入仓已确认',release_and_return:'松爪、退出并返回',hold:'夹紧保持',complete:'流程结束'};
let latestDebug=null,debugSending=false,activeDebugRequest=null;
Object.assign(labels,{chassis_navigation:'保持机械臂，底盘到特殊点',chassis_forward:'底盘前进6秒入仓',release:'入仓后松爪',chassis_retreat:'底盘退回T2',arm_return:'机械臂返回初始姿态'});
labels.lidar_fixed_entry='雷达固定放置（跳过 D405）';labels.finish_travel='补足实测轴向行程';
function debugButtons(){const ready=latestDebug?.can_start===true&&!debugSending;$('locate-only').disabled=!ready;$('start').disabled=!ready||!$('debug-confirm').checked;$('cancel').disabled=!activeDebugRequest||debugSending;}
$('model').hidden=true;$('start').disabled=true;$('start').textContent='2. 机械臂放置并返回';
$('locate-only').onclick=()=>debugAction('observe');$('start').onclick=()=>debugAction('place');$('debug-confirm').onchange=debugButtons;
$('cancel').onclick=()=>activeDebugRequest?debugAction('cancel'):undefined;
async function debugAction(op){if(debugSending)return;debugSending=true;debugButtons();try{const request=op==='cancel'?activeDebugRequest:(crypto.randomUUID?.()||'debug-'+Date.now()+'-'+Math.random().toString(16).slice(2));if(op!=='cancel')activeDebugRequest=request;const body={op,request_id:request};if(op==='place')Object.assign(body,{expected_epoch:latestDebug?.command_epoch,environment_clear:$('debug-confirm').checked,base_fixed_confirmed:$('debug-confirm').checked});const r=await fetch('/api/placement-debug',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const j=await r.json();if(!r.ok)throw Error(j.message||'请求未确认');latestDebug=j;renderDebug(j);}catch(e){$('debug-status').textContent=e.message;}finally{debugSending=false;debugButtons();}}
function renderDebug(j){$('debug-status').textContent=(j.reason||j.status)+(j.reference_reason?'；参考对比：'+j.reference_reason:'');const r=j.residual||{};$('debug-residual').textContent=`视觉偏差：前后 ${typeof r.forward_m==='number'?(r.forward_m*1000).toFixed(1)+' mm':'—'}；左右 ${typeof r.lateral_m==='number'?(r.lateral_m*1000).toFixed(1)+' mm':'—'}；偏航 ${typeof r.yaw_deg==='number'?r.yaw_deg.toFixed(2)+'°':'—'}`;const img=$('debug-observation');img.hidden=!j.image;if(j.image)img.src=j.image;else img.removeAttribute('src');debugButtons();}
async function updateDebug(){try{const r=await fetch('/api/placement-debug');const j=await r.json();if(!r.ok)throw Error(j.message||'读取失败');latestDebug=j;renderDebug(j);}catch(e){latestDebug=null;debugButtons();$('debug-status').textContent=e.message;}setTimeout(updateDebug,600);}
function imageFromComparison(id,item){const node=$(id);if(item&&item.status==='available'&&item.image_file){node.src=item.image_file;node.hidden=false;}else{node.removeAttribute('src');node.hidden=true;}}
function renderComparison(j){const images=j.images||{};imageFromComparison('reference-d435',images.reference_d435);imageFromComparison('current-d435',images.current_d435);const r=j.residual||{};$('comparison-residual').textContent=`偏差：前后 ${typeof r.forward_m==='number'?(r.forward_m*1000).toFixed(1)+' mm':'—'}；左右 ${typeof r.lateral_m==='number'?(r.lateral_m*1000).toFixed(1)+' mm':'—'}；偏航 ${typeof r.yaw_deg==='number'?r.yaw_deg.toFixed(2)+'°':'—'}`;$('reference-status').textContent=j.reference_id?'当前参考：'+j.reference_id:'尚未选择参考';}
async function refreshReference(){try{const r=await fetch('/api/placement-reference');const j=await r.json();if(r.ok)renderComparison(j);else $('reference-status').textContent=j.message||'参考读取失败';}catch(e){$('reference-status').textContent='参考接口暂不可用';}}
$('refresh-reference').onclick=refreshReference;
$('record-reference').onclick=async()=>{const button=$('record-reference');button.disabled=true;try{const r=await fetch('/api/placement-reference',{method:'POST',headers:{'Content-Type':'application/json'},body:'{"op":"record_current"}'});const j=await r.json();$('reference-status').textContent=r.ok?'已保存：'+j.reference_id:(j.message||'保存失败');if(r.ok)await refreshReference();}catch(e){$('reference-status').textContent='保存失败：连接中断';}finally{button.disabled=false;}};
function seconds(value){return typeof value==='number'?value.toFixed(1)+' s':'—';}
function timingRows(j){const rows=[...(j.timeline||[])];if(j.active_phase)rows.push(j.active_phase);const body=$('timings');body.replaceChildren();if(!rows.length){const tr=document.createElement('tr'),td=document.createElement('td');td.colSpan=5;td.textContent='等待任务';tr.append(td);body.append(tr);return;}for(const row of rows){const tr=document.createElement('tr');for(const value of [labels[row.phase]||row.phase,seconds(row.planned_duration_s),seconds(row.duration_s),seconds(row.cumulative_duration_s),row.status==='running'?'进行中':(row.status||'')]){const td=document.createElement('td');td.textContent=value;tr.append(td);}body.append(tr);}}
function signed(value){return typeof value==='number'?(value>=0?'+':'')+value.toFixed(2)+' mm':'—';}
function d405Metrics(j){$('d405-metrics').textContent=`误差 ${signed(j.d405_raw_error_mm)}；步长 ${signed(j.d405_step_mm)}；累计 ${signed(j.d405_cumulative_correction_mm)}；第 ${typeof j.d405_attempt==='number'?j.d405_attempt:'—'} 次`;
  if(j.placement_mode==='lidar_special_point'){$('d405-status').textContent='雷达底盘入仓：D405 对位和坐底确认关闭';$('d405-metrics').textContent='机械臂保持预观察点；特殊点后底盘前进 6 秒，再松爪退出';return;}
  if(j.d405_skipped){$('d405-status').textContent='雷达固定放置：D405 横向对位及坐底确认已关闭';$('d405-metrics').textContent=`轴向实测 ${typeof j.measured_travel_mm==='number'?j.measured_travel_mm.toFixed(1):'—'} / 150 mm；按行程松爪返回`;return;}
  if(j.d405_converged)$('d405-status').textContent='已收敛，使用本次 D405 证据';
  else if(j.d405_recovery)$('d405-status').textContent=j.d405_recovery.message;
  else if(j.d405_degraded)$('d405-status').textContent='D405 等待识别：'+(j.d405_reason||'D405 证据不可用');
  else if(j.d405_attempt)$('d405-status').textContent='正在按新画面左右微调';}
async function update(){try{const r=await fetch('/api/fast-level');const j=await r.json();const status=j.status==='complete_degraded'?'降级结束':j.status==='complete_unverified'?'入仓未确认 · 本轮结束':j.status;$('phase').textContent=(labels[j.phase]||j.phase)+' · '+status;$('reason').textContent=j.reason||j.fallback_reason||'';debugButtons();const total=Number(j.total_duration_s||0).toFixed(1),phase=Number(j.phase_duration_s||0).toFixed(1);$('metrics').textContent=`本步 ${phase}s；总计 ${total}s；`+(j.entry_source?`入口：${j.entry_source==='visual_chassis+d405_preobservation'?'视觉底盘 + D405 预观察位':'固定标定'}；`:'')+(typeof j.depth_mm==='number'?`${j.seat_verification_enabled===false?'行程':'深度'} ${j.depth_mm.toFixed(1)}mm；`:'')+(j.insertion_skipped?'已跳过推入；':'')+(j.released?'已松爪；':'')+(j.grip?'夹持：'+(j.grip.stable?'稳定':'等待'):'');d405Metrics(j);timingRows(j);}catch(e){$('reason').textContent='控制连接中断；请留意现场状态';}setTimeout(update,300);}
async function d435Camera(){try{const o=await(await fetch('/camera-overlay/d435.json')).json();if(!o.stale&&!o.error&&Date.now()/1000-o.updated_at<2){const sequence=encodeURIComponent(o.sequence||o.updated_at);$('d435').src='/camera-overlay/d435.jpg?sequence='+sequence;$('d435-depth').src='/camera-overlay/d435-depth.jpg?sequence='+sequence;$('vision-status').textContent=o.reason||'彩色与同帧深度已更新';}else{const m=await(await fetch('/camera-live/d435.json')).json();const f=m.image_file||m.raw_image_file;if(f&&m.state==='online')$('d435').src='/camera-live/'+encodeURIComponent(f);$('vision-status').textContent=o.error||'点击只做视觉定位，生成本轮识别结果';}}catch(e){$('vision-status').textContent='D435 画面暂时不可用';}}
async function d405Camera(){
  const image=$('d405-horizontal'),status=$('d405-camera-status');
  try{
    const r=await fetch('/camera-overlay/d405-horizontal.json',{cache:'no-store',signal:AbortSignal.timeout(2500)});const o=await r.json();
    if(r.ok&&o.frame_id&&Number.isFinite(o.image_time)){
      const evidence=$('d405-evidence');evidence.src='/camera-overlay/d405-horizontal.jpg?sequence='+encodeURIComponent(o.frame_id+'-'+o.image_time);evidence.hidden=false;
      const age=Math.max(0,Math.round(Date.now()/1000-o.image_time));
      const face=o.battery_visual?.status==='perspective_four_edge_fit';
      const consensus=face?(o.battery_visual?.consensus_required===false?'（有效单帧，已排除把手；多帧一致性仅提示）':o.battery_visual?.consensus_frames?`（${o.battery_visual.sampled_frames} 帧中 ${o.battery_visual.consensus_frames} 帧一致，已排除把手）`:''):'';
      $('d405-evidence-status').textContent=o.method==='left_inner_rim_to_left_battery_edge' ? `标定快照 · ${age} 秒前 · 橙线为仓口左内沿，${o.battery_reference==='side_plane_at_mouth'?'紫点为电池侧面实测深度，间隙按仓口平面估算':'紫线为电池左外沿'} · 间隙 ${o.clearance_mm.toFixed(1)} mm · ${o.in_band?'在 0–5 mm 范围内，保持':('本帧建议沿 '+(o.step_mm>0?'+Y':'−Y')+' 移动 '+Math.abs(o.step_mm).toFixed(1)+' mm')}` : `标定快照 · ${age} 秒前（非实时）· ${o.rim?.status?.startsWith('visible')?'可见仓口边线':'仓口未确认'}；电池表面：${face?'四条透视棱线已拟合，洋红箭头表示朝向'+consensus:'四棱未确认'}；毫米修正：${o.battery_front?.status==='visible'?'已有深度证据':'尚不可用'}`+(o.reason&&!face?' · '+o.reason:'');
    }
    if(!r.ok||!o.frame_id||o.stale||!Number.isFinite(o.image_time)||Math.abs(Date.now()/1000-o.image_time)>2)throw Error('no fresh overlay');
    image.src='/camera-overlay/d405-horizontal.jpg?sequence='+encodeURIComponent(o.frame_id+'-'+o.image_time);image.hidden=false;
    status.textContent='D405 · 本轮水平修正标注';
    if(o.status==='unavailable')$('d405-status').textContent='本帧不可用：'+(o.reason||'可见棱或深度不足');
    return;
  }catch(e){}
  try{
    const r=await fetch('/camera-live/d405.json',{cache:'no-store',signal:AbortSignal.timeout(2500)});const m=await r.json(),file=m.image_file||m.raw_image_file;
    if(!r.ok||m.state!=='online'||!file||!Number.isFinite(m.captured_at)||Math.abs(Date.now()/1000-m.captured_at)>3)throw Error('no fresh camera frame');
    image.src='/camera-live/'+encodeURIComponent(file);image.hidden=false;
    status.textContent='D405 · 实时彩色画面（当前帧修正标注未生成）';
  }catch(e){image.hidden=true;image.removeAttribute('src');status.textContent='D405 · 实时画面暂不可用，等待新帧';}
}
$('refresh-d405').onclick=async()=>{
  const button=$('refresh-d405');button.disabled=true;$('d405-evidence-status').textContent='正在采集配对 RGB-D，剔除背景和把手；有效单帧即可显示，不执行运动';
  try{
    const r=await fetch('/api/d405-placement-preview',{method:'POST',headers:{'Content-Type':'application/json'},body:'{"op":"refresh"}'});const result=await r.json();
    if(!r.ok)throw Error(result.message||'采集失败');await d405Camera();
  }catch(error){$('d405-evidence-status').textContent='标定线刷新失败：'+error.message;}
  finally{button.disabled=false;}
};
async function cameras(){await Promise.all([d435Camera(),d405Camera()]);setTimeout(cameras,900);}update();cameras();updateDebug();
