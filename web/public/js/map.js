const el = id => document.getElementById(id);
const canvas = el('map');
const ctx = canvas.getContext('2d');
const phases = [
  ['head', 'D435 搜索与粗定位'], ['coarse_plan', '粗调规划'],
  ['coarse_execute', '升降与预观察'], ['wrist', 'D405 重新观察'],
  ['preview', '位姿与抓取几何'], ['pick_plan', '抓取规划'],
  ['approach_execute', '预抓接近 / 纠偏'], ['insert', '插入与夹取'],
  ['verify_hold', '夹持核验'], ['extract', '抽出电池']
];
const statuses = {idle:'等待任务', starting:'启动中', running:'运行中', completed:'已完成',
  cancelled:'已取消', needs_attention:'需要处理', awaiting_hold_verification:'等待夹持核验',
  unavailable:'抓取后端未启用', failed:'执行失败'};
let latest = null;
let connected = false;
let received = 0;

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
  const pose = data.pose, grid = data.map, task = data.task || {};
  el('map-info').textContent = grid ? `${grid.frame} · ${(grid.width*grid.resolution).toFixed(1)} × ${(grid.height*grid.resolution).toFixed(1)} m · 显示栅格 ${grid.resolution.toFixed(3)} m` : '地图坐标尚未就绪';
  el('pose-state').textContent = pose?.valid ? '定位反馈有效' : (data.error || '定位数据过期');
  el('pose-value').textContent = pose ? `X ${pose.x.toFixed(3)} m / Y ${pose.y.toFixed(3)} m / 朝向 ${(pose.yaw*180/Math.PI).toFixed(1)}°` : '等待 map → base_link 定位变换';
  el('pose-age').textContent = pose ? `反馈年龄 ${pose.age_s.toFixed(1)} 秒` : '尚无定位反馈';
  el('path-state').textContent = data.path ? `路径反馈 ${data.path.age_s.toFixed(1)} 秒前${data.path.age_s >= 5 ? '，已隐藏过期路径' : ''}` : '等待规划路径';
  el('task-state').textContent = statuses[task.status] || task.status || '等待任务状态';
  el('phase').textContent = phases.find(([key]) => key === task.phase)?.[1] || (task.phase === 'idle' ? '尚未开始' : task.phase) || '—';
  el('mission').textContent = task.mission_stage && task.mission_stage !== 'IDLE' ? `任务状态机：${task.mission_stage}` : ''; 
  el('reason').textContent = task.reason || (task.recovery_required ? '任务要求恢复处理' : '');
  el('run').textContent = task.run_id || '';
  el('stages').replaceChildren(...phases.map(([key, label]) => {
    const li = document.createElement('li'); li.textContent = label;
    // Re-observation loops mean earlier list entries are not necessarily completed.
    if (key === task.phase) { li.className = 'current'; li.setAttribute('aria-current','step'); }
    return li;
  }));
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
