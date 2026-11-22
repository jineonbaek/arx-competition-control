const taskKey=document.body?.dataset?.taskKey || 'competition';
const taskEndpoint=document.body?.dataset?.taskEndpoint || '/api/competition-task';
const el = id => document.getElementById(id);
const canvas = el('map');
const ctx = canvas.getContext('2d');
const statuses = {idle:'等待比赛', running:'比赛运行中', recovering:'Qwen 分析与恢复中',
  stopping:'正在停止并交还控制权', paused:'已停止，可检查后继续',
  completed:'整场比赛完成', needs_attention:'任务中断，需要检查'};
if(taskKey==='t2_test')Object.assign(statuses,{idle:'等待T2实测',running:'T2实测运行中',completed:'T2单独实测完成'});
let latest = null;
let connected = false;
let received = 0;
let taskBusy = false;
let directTask = null;
let directConnected = false;

function renderCompetition(task) {
  task = (taskKey==='t2_test' ? directTask : task) || {};
  const taskConnected=taskKey==='t2_test' ? directConnected : connected;
  el('task-state').textContent=statuses[task.status] || '比赛总控未启用';
  if(taskKey==='t2_test' && task.target)el('target').textContent=task.target;
  el('phase').textContent=(task.tasks || []).find(t=>t.id===task.phase)?.label || '—';
  el('mission').textContent=task.control_owned?'控制权：整场任务管理':
    task.manual_available===false?'控制权：其他任务正在使用':'控制权：9983 手动控制';
  const p=task.phase==='place' && task.placement_progress;
  const lines=[];
  if(p){
    const phases={d405_preobservation:'进入预观测点',j4_sag_check:'检查 J4 下坠',d405_horizontal_trim:'D405 水平对齐',seat_push:'沿电池轴推入',compensate:'末段补推',verify:'D405 确认坐底',release:'松爪',exit:'原路返回',release_and_return:'松爪并返回'};
    if(phases[p.phase])lines.push(phases[p.phase]);
    if(Number.isFinite(p.measured_travel_mm))lines.push('实测推进 '+p.measured_travel_mm.toFixed(1)+' mm');
    if(p.d405_seat_attempt)lines.push('坐底复查 '+p.d405_seat_attempt+' 次');
    else if(p.d405_attempt)lines.push('水平观测 '+p.d405_attempt+' 次');
    if(Number.isFinite(p.d405_live?.remaining_mm))lines.push('D405 估计剩余 '+p.d405_live.remaining_mm.toFixed(1)+' mm');
    if(p.reason || p.d405_reason || p.d405_live?.reason)lines.push(p.reason || p.d405_reason || p.d405_live.reason);
  }
  el('reason').textContent=[task.reason || task.placement_readiness?.reason,...lines].filter(Boolean).join(' · ');
  const nav=task.navigation, handoff=nav?.handoff;
  el('navigation-state').textContent=nav?`导航：${nav.route?.available?nav.route.status || '等待路线':nav.error || '导航执行端未就绪'}${nav.route?.drive_status?' · '+nav.route.drive_status:''}`:'';
  el('navigation-handoff').textContent=nav?`交接：${handoff?.task?handoff.task.point_id+' · '+handoff.task.status:nav.registered?'等待到点通知':'比赛开始时登记'}${handoff?.navigation_check?.reason?' · '+handoff.navigation_check.reason:''}`:'';
  el('run').textContent=task.run_id || '';
  el('competition-start').disabled=taskBusy || !taskConnected || !task.can_start || !el('environment-clear').checked;
  el('competition-continue').disabled=taskBusy || !taskConnected || !task.can_continue || !el('environment-clear').checked;
  el('competition-stop').disabled=!task.control_owned && !['running','recovering','stopping'].includes(task.status);
  const pickupMode=el('competition-mode');
  if(pickupMode){
    pickupMode.disabled=taskBusy || !taskConnected || !task.can_start;
    if(task.control_owned)pickupMode.value=task.mode || 'stable';
    renderCompetitionMode();
  }
  el('stages').replaceChildren(...(task.tasks || []).map(t=>{
    const li=document.createElement('li');
    const label={completed:'完成',running:'执行中',stopped:'已停止',failed:'需要检查',pending:'待执行'};
    li.textContent=`${t.label} · ${!t.connected?'能力未接入':label[t.status] || t.status}`;
    if(t.status==='completed')li.className='done';
    else if(t.id===task.phase){li.className='current';li.setAttribute('aria-current','step');}
    return li;
  }));
  el('task-logs').replaceChildren(...(task.logs || []).slice(-20).map(item=>{
    const li=document.createElement('li');
    const events={start:'比赛开始',continue:'检查后继续',stop_requested:'请求停止',module_start:'开始子任务',
      module_inspect:'核对原任务',module_completed:'子任务完成',recovery_analysis:'异常分析',
      recovery_decision:'恢复决策',current_state_checked:'读取当前状态',competition_completed:'比赛完成',
      competition_paused:'已停止',competition_needs_attention:'任务中断'};
    li.textContent=`${new Date(item.time*1000).toLocaleTimeString('zh-CN',{hour12:false})} ${events[item.event] || item.event} ${item.detail}`;
    return li;
  }));
}

async function competitionAction(action) {
  if(action!=='stop' && taskBusy)return;
  const request_id=Array.from(crypto.getRandomValues(new Uint8Array(16)),b=>b.toString(16).padStart(2,'0')).join('');
  const data={action,request_id};
  if(action==='start' && taskKey==='competition' && el('competition-mode')?.value==='fast')data.mode='fast';
  if(action!=='stop')Object.assign(data,{expected_epoch:(taskKey==='t2_test' ? directTask : latest?.[taskKey])?.command_epoch,
    environment_clear:el('environment-clear').checked});
  taskBusy=true;renderCompetition(latest?.[taskKey]);el('task-message').textContent='';
  try {
    const response=await fetch(taskEndpoint,{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify(data),signal:AbortSignal.timeout(10000)});
    const task=await response.json();
    if(!response.ok)throw new Error(task.message || '比赛操作未被接受');
    if(taskKey==='t2_test')directTask=task;
    else if(latest)latest[taskKey]=task;
  } catch(error) {el('task-message').textContent=error.message+'；请查看当前状态，勿重复启动任务。';}
  finally {taskBusy=false;renderCompetition(latest?.[taskKey]);}
}
for(const action of ['start','stop','continue'])el('competition-'+action).addEventListener('click',()=>competitionAction(action));
function renderCompetitionMode(){
  const description=el('competition-mode-description');
  if(description)description.textContent=el('competition-mode')?.value==='fast'
    ?'启动时右臂到已保存的快模式预抓点；到点后 D405 识别并精纠。后退 200 mm 后交给运输，导航时同时降升降和返回运输姿态。'
    :'稳模式沿用原比赛流程。';
}
el('competition-mode')?.addEventListener('change',renderCompetitionMode);
el('environment-clear').addEventListener('change',()=>renderCompetition(latest?.[taskKey]));

function draw() {
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.round(canvas.clientWidth * ratio);
  canvas.height = Math.round(canvas.clientHeight * ratio);
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  const width = canvas.clientWidth, height = canvas.clientHeight;
  ctx.clearRect(0, 0, width, height);
  const grid = latest?.map;
  el('empty').hidden = !!grid;
  if (!grid) return;
  const scale = Math.min((width - 36) / grid.width, (height - 36) / grid.height);
  const left = (width - grid.width * scale) / 2, top = (height - grid.height * scale) / 2;
  const raster = document.createElement('canvas');
  raster.width = grid.width; raster.height = grid.height;
  const rc = raster.getContext('2d'), pixels = rc.createImageData(grid.width, grid.height);
  grid.cells.forEach((value, i) => {
    const x = i % grid.width, y = Math.floor(i / grid.width);
    const offset = ((grid.height - 1 - y) * grid.width + x) * 4;
    const shade = value < 0 ? 180 : Math.round(248 - 205 * value / 100);
    pixels.data.set([shade, shade, shade, 255], offset);
  });
  rc.putImageData(pixels, 0, 0); ctx.imageSmoothingEnabled = false;
  ctx.drawImage(raster, left, top, grid.width * scale, grid.height * scale);
  const [ox, oy, angle] = grid.origin, c = Math.cos(angle), s = Math.sin(angle);
  function project(x, y) {
    const dx = x - ox, dy = y - oy;
    return [left + (c * dx + s * dy) / grid.resolution * scale,
      top + (grid.height - (-s * dx + c * dy) / grid.resolution) * scale];
  }
  const path = latest.path;
  if (connected && path && path.frame === grid.frame && path.age_s + (performance.now()-received)/1000 < 5) {
    ctx.beginPath(); ctx.strokeStyle = '#1380b6'; ctx.lineWidth = 2.5;
    path.points.forEach(([x,y], i) => { const p = project(x,y); i ? ctx.lineTo(...p) : ctx.moveTo(...p); });
    ctx.stroke();
  }
  const pose = latest.pose;
  if (pose && pose.frame === grid.frame) {
    const [x,y] = project(pose.x, pose.y);
    ctx.save(); ctx.translate(x,y); ctx.rotate(-(pose.yaw - angle));
    ctx.fillStyle = connected && pose.valid && pose.age_s + (performance.now()-received)/1000 <= 2 ? '#e97822' : '#75818c';
    ctx.beginPath(); ctx.moveTo(13,0); ctx.lineTo(-9,-8); ctx.lineTo(-5,0); ctx.lineTo(-9,8); ctx.closePath(); ctx.fill(); ctx.restore();
  }
}

function render(data) {
  const pose = data.pose, grid = data.map;
  el('map-info').textContent = grid ? `${grid.frame} · ${(grid.width*grid.resolution).toFixed(1)} × ${(grid.height*grid.resolution).toFixed(1)} m · 显示栅格 ${grid.resolution.toFixed(3)} m` : '地图坐标尚未就绪';
  el('pose-state').textContent = pose?.valid ? '定位反馈有效' : (data.error || '定位数据过期');
  el('pose-value').textContent = pose ? `X ${pose.x.toFixed(3)} m / Y ${pose.y.toFixed(3)} m / 朝向 ${(pose.yaw*180/Math.PI).toFixed(1)}°` : '等待 map → base_link 定位变换';
  el('pose-age').textContent = pose ? `反馈年龄 ${pose.age_s.toFixed(1)} 秒` : '尚无定位反馈';
  el('path-state').textContent = data.path ? `路径反馈 ${data.path.age_s.toFixed(1)} 秒前${data.path.age_s >= 5 ? '，已隐藏过期路径' : ''}` : '等待规划路径';
  renderCompetition(data[taskKey]);
  draw();
}

async function poll() {
  try {
    const requested = performance.now();
    const response = await fetch('/api/navigation-view', {signal:AbortSignal.timeout(10000), cache:'no-store'});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    latest = await response.json();
    if (latest.map?.runs) {
      const cells = [];
      for (let i=0; i<latest.map.runs.length; i+=2) {
        const value=latest.map.runs[i], count=latest.map.runs[i+1];
        for (let j=0; j<count; j++) cells.push(value);
      }
      latest.map.cells = cells;
    }
    connected = true; received = performance.now();
    const transferAge = (received-requested)/1000;
    if (latest.pose) {
      latest.pose.age_s += transferAge;
      latest.pose.valid = latest.pose.valid && latest.pose.age_s <= 2;
    }
    if (latest.path) latest.path.age_s += transferAge;
    el('connection').textContent = '实时连接 · 每秒更新'; render(latest);
  } catch (error) {
    connected = false; el('connection').textContent = `连接中断 · ${error.message}`;
    el('pose-state').textContent = '定位反馈已过期';
    el('task-state').textContent = '连接中断，以下为最后收到的任务状态'; draw();
    renderCompetition(latest?.[taskKey]);
    if(taskKey!=='t2_test')el('task-state').textContent='连接中断，以下为最后收到的任务状态';
  } finally { setTimeout(poll, 1000); }
}
setInterval(() => {
  if (latest?.pose) {
    const age = latest.pose.age_s + (performance.now()-received)/1000;
    el('pose-age').textContent = `反馈年龄 ${age.toFixed(1)} 秒`;
    if (age <= 2) return;
    el('pose-state').textContent = '定位反馈已过期'; draw();
  }
}, 500);
new ResizeObserver(draw).observe(canvas.parentElement);
poll();

async function pollDirectTask() {
  try {
    const response=await fetch(taskEndpoint,{signal:AbortSignal.timeout(4000),cache:'no-store'});
    if (!response.ok) throw new Error('HTTP '+response.status);
    directTask=await response.json();
    directConnected=true;
  } catch {
    directConnected=false;
  }
  renderCompetition(directTask);
  if (!directConnected) el('task-state').textContent='T2 服务连接中断';
  setTimeout(pollDirectTask,1000);
}
if (taskKey==='t2_test') pollDirectTask();
