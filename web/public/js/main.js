import { newEditor, followFeedback, checkedNumber, validBounds, CHASSIS_KEYS, chassisKeyIntent } from './state.js';

const $ = id => document.getElementById(id);
const armName = side => side === 'left' ? '左臂' : '右臂';
const fmt = (v, digits = 2) => Number.isFinite(v) ? v.toFixed(digits) : '—';
let snapshot = null;
let connected = false;
let editor = newEditor('left', null);
let liftDraft = null;
let liftDirty = false;
let held = null;
let motionMode = false;
const pressedDriveKeys = new Set();
let keyBlocked = false;
let heartbeat = null;
let heartbeatBusy = false;
let baseSequence = 0;
const baseControlId = Array.from(crypto.getRandomValues(new Uint8Array(16)), b=>b.toString(16).padStart(2,'0')).join('');
let lastLogKey = '';
let localLogId = 0;
let toastTimer;
let modeBusy = false;
const localLogs = [];
const jointInputs = [], jointSliders = [], jointErrors = [], jointActual = [], jointRanges = [];
const tcpInputs = [];

function toast(text) {
  $('toast').textContent = text;
  $('toast').classList.add('visible');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => $('toast').classList.remove('visible'), 4500);
}
function localLog(text, level = 'info') {
  localLogs.push({ id: 'local-' + (++localLogId), time: Date.now() / 1000, text, level });
  if (localLogs.length > 30) localLogs.shift();
  renderLogs();
}
function renderLogs() {
  const entries = [...(snapshot?.logs || []).map(x=>({...x,source:'后端'})),
                   ...localLogs.map(x=>({...x,source:'网页'}))].sort((a,b) => a.time-b.time).slice(-100);
  const key = JSON.stringify(entries);
  if (key === lastLogKey) return;
  lastLogKey = key;
  $('log-count').textContent='最近 '+entries.length+' 条';
  $('log-area').replaceChildren();
  if (!entries.length) {
    const p = document.createElement('p'); p.className = 'log-empty'; p.textContent = '等待指令或连接事件';
    $('log-area').append(p);
  }
  for (const entry of entries) {
    const row = document.createElement('div'); row.className = 'log-line ' + (entry.level || '');
    const time = document.createElement('span'); time.className = 'log-time';
    time.textContent = new Date(entry.time * 1000).toLocaleTimeString('zh-CN', { hour12:false });
    const source=document.createElement('span');source.className='log-source';source.textContent=entry.source;
    row.append(time, source, document.createTextNode(entry.text)); $('log-area').append(row);
  }
  if($('log-autoscroll').checked)$('log-area').scrollTop = $('log-area').scrollHeight;
}
function setLogOpen(open) {
  if(open)exitMotionMode();
  $('log-panel').hidden=!open;
  $('btn-log-toggle').setAttribute('aria-expanded',String(open));
  $('btn-log-toggle').classList.toggle('active',open);
  if(open && $('log-autoscroll').checked)$('log-area').scrollTop=$('log-area').scrollHeight;
  if(!open)$('btn-log-toggle').focus();
}
$('btn-log-toggle').addEventListener('click',()=>setLogOpen($('log-panel').hidden));
$('btn-log-close').addEventListener('click',()=>setLogOpen(false));
document.addEventListener('keydown',event=>{
  if(event.key==='Escape' && !$('log-panel').hidden){event.preventDefault();setLogOpen(false);}
});
function setInput(input, value, digits, force = false) {
  if (!force && document.activeElement === input) return;
  input.value = Number.isFinite(value) ? value.toFixed(digits) : '';
}
function rangeInput(slider, pair, value) {
  if (!validBounds(pair)) { slider.disabled = true; return; }
  slider.min = String(pair[0]); slider.max = String(pair[1]);
  slider.disabled = false;
  if (Number.isFinite(value)) slider.value = String(value);
}
function markValidity(input, error, pair) {
  const checked = checkedNumber(input.value, pair?.[0] ?? -Infinity, pair?.[1] ?? Infinity);
  input.setCustomValidity(checked.message);
  error.textContent = checked.message;
  input.closest('.joint-row')?.classList.toggle('control-invalid', !checked.valid);
  return checked;
}

for (let i=0; i<6; i++) {
  const row = document.createElement('div'); row.className = 'joint-row';
  row.innerHTML = '<div class="joint-label"><label class="joint-name" for="joint-input-'+i+'">J'+(i+1)+'<small>目标角度</small></label><div class="number-wrap"><input id="joint-input-'+i+'" type="number" step="0.1" disabled><span>°</span></div></div><input id="joint-slider-'+i+'" type="range" class="joint-slider" min="0" max="1" value="0.5" step="0.1" aria-label="J'+(i+1)+'目标角度滑块" disabled><div class="joint-meta"><span class="joint-range">安全范围待确认</span><span class="actual-value">实际 —</span></div><div class="control-validation" role="alert"></div>';
  $('joint-controls-body').append(row);
  const input = row.querySelector('input[type=number]'), slider = row.querySelector('input[type=range]');
  const error = row.querySelector('.control-validation');
  jointInputs.push(input); jointSliders.push(slider); jointErrors.push(error);
  jointActual.push(row.querySelector('.actual-value')); jointRanges.push(row.querySelector('.joint-range'));
  function edit(raw, fromSlider) {
    if (!editor.joints) return;
    editor.dirty.joints = true;
    editor.joints[i] = String(raw).trim() === '' ? NaN : Number(raw);
    if (fromSlider) setInput(input, editor.joints[i], 1, true);
    const pair = snapshot?.limits.arms[editor.side].joint_limits_deg?.[i];
    const checked = markValidity(input, error, pair);
    if (checked.valid) slider.value = String(checked.value);
    $('joint-mode').textContent = '目标编辑';
  }
  slider.addEventListener('input', () => edit(slider.value, true));
  input.addEventListener('input', () => edit(input.value, false));
  input.addEventListener('change', () => {
    if (markValidity(input, error, snapshot?.limits.arms[editor.side].joint_limits_deg?.[i]).valid)
      setInput(input, editor.joints[i], 1, true);
  });
}

['X','Y','Z','Roll','Pitch','Yaw'].forEach((axis, i) => {
  const field = document.createElement('div'); field.className = 'tcp-field';
  field.innerHTML = '<label for="tcp-'+i+'">'+axis+'</label><div class="number-wrap"><input id="tcp-'+i+'" type="number" step="0.1" disabled><span>'+(i<3?'mm':'°')+'</span></div>';
  $('tcp-controls').append(field); const input=field.querySelector('input'); tcpInputs.push(input);
  input.addEventListener('input', () => {
    if (!editor.tcp) return;
    editor.dirty.tcp=true; editor.tcp[i]=input.value.trim()==='' ? NaN:Number(input.value);
    $('tcp-mode').textContent='目标编辑';
    const values=tcpInputs.map(x=>checkedNumber(x.value)); $('tcp-error').textContent=values.every(x=>x.valid)?'':'末端位姿需要六个有效数值';
  });
});

function selectArm(side) {
  editor=newEditor(side,snapshot?.arms[side]);
  for (const input of [...jointInputs,...tcpInputs,$('gripper-input')]) input.setCustomValidity('');
  for (const error of [...jointErrors,$('gripper-error'),$('tcp-error')]) error.textContent='';
  render(true);
}
$('select-left').addEventListener('click',()=>selectArm('left'));
$('select-right').addEventListener('click',()=>selectArm('right'));
$('restore-feedback').addEventListener('click',()=>selectArm(editor.side));
function status(id, ok, text) { $(id).classList.toggle('online',ok); $(id).lastChild.textContent=text; }
function button(id, key) {
  const reason = !connected ? '网页连接已断开' : snapshot?.capabilities[key] ?? '等待反馈';
  $(id).disabled=Boolean(reason); $(id).title=reason;
}
function render(force=false) {
  if (!snapshot) return;
  const side=editor.side, name=armName(side), arm=snapshot.arms[side], limits=snapshot.limits.arms[side];
  $('select-left').classList.toggle('selected',side==='left'); $('select-right').classList.toggle('selected',side==='right');
  $('select-left').setAttribute('aria-selected',String(side==='left')); $('select-right').setAttribute('aria-selected',String(side==='right'));
  $('arm-stop').textContent='停止'+name; $('joint-send').textContent='发送到'+name;
  status('web-status',connected,connected?'网页已连接':'网页已断开');
  status('ros-status',snapshot.ros.connected,snapshot.ros.connected?'ROS 已连接':'ROS 未连接');
  status('arm-feedback-status',connected && arm.fresh,connected && arm.fresh?name+'实时反馈':name+'反馈缺失或过期');
  $('mode-status').textContent=snapshot.execution_enabled?'控制模式':'观察模式'; $('mode-status').classList.toggle('enabled',snapshot.execution_enabled);
  $('notice').textContent=snapshot.execution_enabled?'真实控制模式：仅当反馈和驱动就绪时开放对应按钮。请核对所选左右臂、目标与现场状态。':'当前为观察模式：反馈可查看，真实运动入口尚未启用。';
  $('notice').classList.toggle('enabled',snapshot.execution_enabled);
  $('arm-reason').textContent=snapshot.capabilities[side+':arm.joints'];
  const feedbackSource=snapshot.ros.feedback_sources?.[side];
  $('arm-feedback-detail').textContent=!snapshot.ros.connected?'ROS 未连接，无法接收机械臂反馈。':
    feedbackSource?.message || (arm.fresh?'正在接收实际关节与末端反馈。':'尚未收到有效反馈；可打开日志查看连接状态。');

  $('arm-mode-target').textContent='作用于'+name;
  for(const el of document.querySelectorAll('[data-arm-mode]')){
    button(el.id,side+':arm.mode.'+el.dataset.armMode);
    el.disabled ||= modeBusy;
  }
  $('joint-mode').textContent=editor.dirty.joints?'目标编辑':'实时跟随';
  $('tcp-mode').textContent=editor.dirty.tcp?'目标编辑':'mm / °';
  for(let i=0;i<6;i++) {
    const pair=limits.joint_limits_deg?.[i];
    jointInputs[i].disabled=!(connected && arm.fresh && validBounds(pair));
    if(validBounds(pair)){jointInputs[i].min=pair[0];jointInputs[i].max=pair[1];}
    setInput(jointInputs[i],editor.joints?.[i],1,force);
    rangeInput(jointSliders[i],pair,editor.joints?.[i]); jointSliders[i].disabled ||= jointInputs[i].disabled;
    jointActual[i].textContent='实际 '+fmt(arm.fresh?arm.joints?.[i]*180/Math.PI:NaN,1)+'°';
    jointRanges[i].textContent=validBounds(pair)?pair[0]+'° ～ '+pair[1]+'°':'安全范围待确认';
  }
  const gp=limits.gripper_limits_raw;
  $('gripper-input').disabled=!(connected && arm.fresh && validBounds(gp));
  if(validBounds(gp)){$('gripper-input').min=gp[0];$('gripper-input').max=gp[1];}
  setInput($('gripper-input'),editor.gripper,2,force);
  rangeInput($('gripper-slider'),gp,editor.gripper); $('gripper-slider').disabled ||= $('gripper-input').disabled;
  $('gripper-actual').textContent='实际 '+fmt(arm.fresh?arm.gripper:NaN)+' · SDK 原值';
  $('gripper-range').textContent=validBounds(gp)?gp.join(' ～ ')+' · SDK 原值':'夹爪端点待确认';
  tcpInputs.forEach((input,i)=>{
    input.disabled=!(connected && arm.base_wrist);
    setInput(input,editor.tcp?.[i],1,force);
  });
  $('tcp-actual').textContent=arm.base_wrist?'腕部 XYZ '+arm.base_wrist.slice(0,3).map(v=>fmt(v,1)).join(' / ')+' mm':'实际位姿 —';
  for(const [id,type] of [['joint-send','arm.joints'],['tcp-send','arm.wrist'],['gripper-send','gripper'],['arm-stop','arm.stop']])button(id,side+':'+type);
  for(const [id,value] of [['gripper-open',limits.gripper_open_raw],['gripper-close',limits.gripper_closed_raw]]){
    button(id,side+':gripper'); $(id).disabled ||= !Number.isFinite(value);
  }

  button('chassis-stop','chassis.stop');
  $('chassis-motion-toggle').disabled=!motionMode && !chassisReady();
  $('chassis-motion-toggle').textContent=motionMode?'退出运动模式':'运动模式';
  $('chassis-motion-toggle').setAttribute('aria-pressed',String(motionMode));
  $('chassis-motion-toggle').classList.toggle('active',motionMode);
  const intent=chassisKeyIntent(pressedDriveKeys);
  for(const key of document.querySelectorAll('[data-drive-key]')){
    key.classList.toggle('active',motionMode && pressedDriveKeys.has(key.dataset.driveKey));
  }
  $('chassis-reason').textContent=snapshot.capabilities.chassis;
  $('chassis-state').textContent=!motionMode?'键盘未启用':intent.kind==='brake'?'刹车请求':keyBlocked?'等待松键':held?held.label+'指令':'键盘已启用';
  $('keyboard-feedback').textContent=!motionMode?'点击运动模式后才响应键盘。':keyBlocked?'已请求停车；松开全部键后重新按下方向键。':held?'键盘请求：'+held.keys+' · '+held.label:'可组合平移和转向，松开全部方向键停车。';
  $('linear-speed').disabled=Boolean(held);$('angular-speed').disabled=Boolean(held);
  if(Number.isFinite(snapshot.limits.linear))$('linear-speed').max=snapshot.limits.linear;
  if(Number.isFinite(snapshot.limits.angular))$('angular-speed').max=snapshot.limits.angular;
  $('chassis-speed-range').textContent=`平移：0 < 值 ≤ ${snapshot.limits.linear}；转向：0 < 值 ≤ ${snapshot.limits.angular}（SDK 原值）。编辑后重新开启运动模式生效。`;
  $('wheel-feedback').textContent=snapshot.body.fresh?snapshot.body.wheels.map(x=>fmt(x)).join(' / ')+' rad/s':'—';
  $('lift-actual').replaceChildren(document.createTextNode(fmt(snapshot.body.fresh?snapshot.body.height:NaN,3)+' '));
  const unit=document.createElement('small');unit.textContent='SDK rad';$('lift-actual').append(unit);
  if(validBounds(snapshot.limits.lift)){
    $('lift-input').min=snapshot.limits.lift[0];$('lift-input').max=snapshot.limits.lift[1];
    $('lift-range').textContent='软件范围 '+snapshot.limits.lift.join(' ~ ')+' rad · 零位与物理高度尚需确认';
  }
  $('lift-input').disabled=!(connected && snapshot.body.fresh && validBounds(snapshot.limits.lift));
  setInput($('lift-input'),liftDraft,2,force);
  rangeInput($('lift-slider'),snapshot.limits.lift,liftDraft);$('lift-slider').disabled ||= $('lift-input').disabled;
  $('lift-reason').textContent=snapshot.capabilities.lift;
  button('lift-send','lift');button('lift-stop','lift.stop');
  const times=[snapshot.body,...Object.values(snapshot.arms)].filter(x=>x.fresh).map(x=>x.timestamp);
  $('last-update').textContent=times.length?'最近反馈 '+new Date(Math.max(...times)*1000).toLocaleTimeString('zh-CN',{hour12:false}):'等待实时反馈';
  renderLogs();
}
async function command(data, quiet=false) {
  if(data.type==='chassis' || data.type==='chassis.stop')data={...data,control_id:baseControlId};
  if(!quiet)localLog('发送请求：'+JSON.stringify(data));
  try {
    const response=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data),keepalive:true});
    const result=await response.json();
    if(!response.ok || !result.ok)throw new Error(result.message || '指令被拒绝');
    if(!quiet){toast(result.message);localLog('后端响应：'+result.message);}
    return true;
  } catch(error) {if(!quiet){toast(error.message);localLog(error.message,'warning');}return false;}
}
function restoreGroup(side,group) {
  if(editor.side!==side)return;
  editor.dirty[group]=false;followFeedback(editor,snapshot?.arms[side]);render(true);
}
$('joint-send').addEventListener('click',async()=>{
  if(!jointInputs.every((input,i)=>markValidity(input,jointErrors[i],snapshot.limits.arms[editor.side].joint_limits_deg?.[i]).valid))return;
  const side=editor.side;
  await command({type:'arm.joints',arm:side,joints:jointInputs.map(x=>Number(x.value))});
});
$('tcp-send').addEventListener('click',async()=>{
  if(!tcpInputs.every(x=>checkedNumber(x.value).valid)){$('tcp-error').textContent='末端位姿需要六个有效数值';return;}
  const side=editor.side,values=tcpInputs.map(x=>Number(x.value));
  if(await command({type:'arm.wrist',frame:'base_link',arm:side,position_mm:values.slice(0,3),orientation_deg:values.slice(3)}))restoreGroup(side,'tcp');
});
$('arm-stop').addEventListener('click',()=>command({type:'arm.stop',arm:editor.side}));
for(const el of document.querySelectorAll('[data-arm-mode]')){
  el.addEventListener('click',async()=>{
    if(el.disabled || modeBusy)return;
    const side=editor.side,mode=Number(el.dataset.armMode);
    modeBusy=true;render();
    try{
      await command({type:'arm.mode',arm:side,mode});
    }finally{
      modeBusy=false;render();
    }
  });
}
function editGripper(fromSlider) {
  const input=$('gripper-input');
  if(fromSlider)setInput(input,Number($('gripper-slider').value),2,true);
  editor.dirty.gripper=true;editor.gripper=input.value.trim()===''?NaN:Number(input.value);
  const result=markValidity(input,$('gripper-error'),snapshot?.limits.arms[editor.side].gripper_limits_raw);
  if(result.valid)$('gripper-slider').value=String(result.value);
}
$('gripper-input').addEventListener('input',()=>editGripper(false));
$('gripper-slider').addEventListener('input',()=>editGripper(true));
async function sendGripper(value) {
  const side=editor.side,pair=snapshot?.limits.arms[side].gripper_limits_raw;
  const checked=checkedNumber(value,pair?.[0],pair?.[1]);
  if(!checked.valid){$('gripper-error').textContent=checked.message;return;}
  editor.dirty.gripper=true;editor.gripper=checked.value;
  setInput($('gripper-input'),checked.value,2,true);$('gripper-slider').value=String(checked.value);
  $('gripper-input').setCustomValidity('');$('gripper-error').textContent='';render();
  await command({type:'gripper',arm:side,position:checked.value});
}
$('gripper-send').addEventListener('click',()=>sendGripper($('gripper-input').value));
$('gripper-open').addEventListener('click',()=>sendGripper(snapshot.limits.arms[editor.side].gripper_open_raw));
$('gripper-close').addEventListener('click',()=>sendGripper(snapshot.limits.arms[editor.side].gripper_closed_raw));
function editLift(fromSlider) {
  if(fromSlider)setInput($('lift-input'),Number($('lift-slider').value),2,true);
  liftDirty=true;liftDraft=$('lift-input').value.trim()===''?NaN:Number($('lift-input').value);
  if(markValidity($('lift-input'),$('lift-error'),snapshot?.limits.lift).valid)$('lift-slider').value=String(liftDraft);
}
$('lift-input').addEventListener('input',()=>editLift(false));
$('lift-slider').addEventListener('input',()=>editLift(true));
$('lift-restore').addEventListener('click',()=>{liftDirty=false;liftDraft=snapshot?.body.fresh?snapshot.body.height:null;$('lift-error').textContent='';render(true);});
$('lift-send').addEventListener('click',async()=>{
  if(!markValidity($('lift-input'),$('lift-error'),snapshot?.limits.lift).valid)return;
  if(await command({type:'lift',target:Number($('lift-input').value)})){liftDirty=false;liftDraft=snapshot.body.height;render(true);}
});
$('lift-stop').addEventListener('click',()=>command({type:'lift.stop'}));


function chassisReady() {
  return connected && snapshot?.execution_enabled===true &&
    snapshot.capabilities.chassis==='' && snapshot.capabilities['chassis.stop']==='';
}
function editable(target) {
  return target instanceof Element && Boolean(target.closest('input,textarea,select,[contenteditable]:not([contenteditable="false"]),[role="textbox"]'));
}
async function driveTick() {
  if(!held || !motionMode || heartbeatBusy)return;
  heartbeatBusy=true;
  const data={type:'chassis',x:held.x,y:held.y,omega:held.omega,sequence:++baseSequence};
  const ok=await command(data,true);
  heartbeatBusy=false;
  if(!ok){
    localLog('底盘请求失败，已退出运动模式；请查看反馈。','warning');
    exitMotionMode();
  }
}
function startDrive(intent) {
  if(!motionMode || !chassisReady()){exitMotionMode();return;}
  const speed=checkedNumber($('linear-speed').value,Number.MIN_VALUE,snapshot?.limits.linear??0);
  const turn=checkedNumber($('angular-speed').value,Number.MIN_VALUE,snapshot?.limits.angular??0);
  if(!speed.valid || !turn.valid){toast('请检查底盘速度输入');exitMotionMode();return;}
  const linearScale=speed.value/Math.max(1,Math.hypot(intent.x,intent.y));
  held={x:intent.x*linearScale,y:intent.y*linearScale,omega:intent.omega*turn.value,
        keys:intent.codes.map(code=>code.slice(3)).join('+'),
        label:intent.codes.map(code=>CHASSIS_KEYS[code].label).join(' + ')};
  driveTick();heartbeat??=setInterval(driveTick,100);
}
function stopDrive(force=false) {
  const wasHeld=Boolean(held);
  held=null;clearInterval(heartbeat);heartbeat=null;
  if(wasHeld || force)command({type:'chassis.stop',sequence:++baseSequence},true);
}
function exitMotionMode() {
  const wasEnabled=motionMode;
  motionMode=false;pressedDriveKeys.clear();keyBlocked=false;
  stopDrive(wasEnabled);render();
}
function updateKeyboardDrive(forceStop=false) {
  const intent=chassisKeyIntent(pressedDriveKeys);
  if(!pressedDriveKeys.size)keyBlocked=false;
  if(intent.kind==='brake' || intent.kind==='conflict')keyBlocked=true;
  if(intent.kind==='move' && !keyBlocked)startDrive(intent);
  else stopDrive(forceStop);
  render();
}
$('chassis-motion-toggle').addEventListener('click',()=>{
  if(motionMode){exitMotionMode();return;}
  if(!chassisReady())return;
  motionMode=true;pressedDriveKeys.clear();keyBlocked=false;render();
});
$('chassis-stop').addEventListener('click',()=>{
  keyBlocked=pressedDriveKeys.size>0;
  stopDrive(true);render();
});
document.addEventListener('keydown',event=>{
  if(!motionMode)return;
  if(event.ctrlKey || event.altKey || event.metaKey || event.isComposing || editable(event.target)){
    exitMotionMode();return;
  }
  if(!(Object.hasOwn(CHASSIS_KEYS,event.code)) && event.code!=='Space')return;
  event.preventDefault();
  if(event.repeat || pressedDriveKeys.has(event.code))return;
  pressedDriveKeys.add(event.code);
  updateKeyboardDrive(event.code==='Space');
});
document.addEventListener('keyup',event=>{
  if(!motionMode || !pressedDriveKeys.has(event.code))return;
  event.preventDefault();pressedDriveKeys.delete(event.code);updateKeyboardDrive();
});
document.addEventListener('focusin',event=>{if(motionMode && editable(event.target))exitMotionMode();});
window.addEventListener('blur',exitMotionMode);
document.addEventListener('visibilitychange',()=>{if(document.hidden)exitMotionMode();});
window.addEventListener('pagehide',exitMotionMode);
function receive(data) {
  snapshot=data;
  if(motionMode && !chassisReady())exitMotionMode();
  followFeedback(editor,data.arms[editor.side]);
  if(!liftDirty && data.body.fresh)liftDraft=data.body.height;
  render();
}
fetch('/api/state').then(r=>r.json()).then(receive).catch(()=>localLog('网页后端暂不可达','warning'));
const events=new EventSource('/events');
events.onopen=()=>{connected=true;localLog('网页反馈连接已建立');render();};
events.onmessage=event=>{try{receive(JSON.parse(event.data));}catch(error){localLog('反馈数据读取失败','warning');}};
events.onerror=()=>{
  if(connected)localLog('网页反馈连接中断，控制已停用','warning');
  connected=false;exitMotionMode();
};
