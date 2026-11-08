import {frameFresh, cameraImagePath} from './camera-freshness.js';

const byId = id => document.getElementById(id);
const side = byId('handeye-arm');
let preview = byId('d405-preview');
let stopped = false, displayed = null, cameraClock = null, stateClock = null, snapshot = null;
const wait = ms => new Promise(resolve => setTimeout(resolve, ms));
const serverNow = clock => clock ? clock.server + performance.now() - clock.local : NaN;

async function readJSON(path) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), 1800);
  try {
    const response = await fetch(path, {cache: 'no-store', signal: controller.signal});
    if (!response.ok) throw new Error('读取失败');
    const data = await response.json();
    const clock = {server: Date.parse(response.headers.get('Date')), local: performance.now()};
    if (!Number.isFinite(clock.server)) throw new Error('服务器时间不可用');
    return {data, clock};
  } finally { clearTimeout(timer); }
}

function cameraUnavailable() {
  preview.hidden = true;
  byId('d405-message').hidden = false;
  byId('d405-message').textContent = 'D405 画面离线或已过期';
  byId('d405-status').textContent = '画面不可用';
  byId('d405-status').classList.remove('online');
  byId('d405-detail').textContent = '等待新鲜彩色帧';
}

async function updateCamera() {
  try {
    const {data: meta, clock} = await readJSON('/camera-live/d405.json');
    cameraClock = clock;
    if (!frameFresh(meta, serverNow(clock))) return cameraUnavailable();
    const path = cameraImagePath('d405', meta);
    if (path === '/camera-live/d405.jpg') return cameraUnavailable();
    if (displayed?.capture_session === meta.capture_session && displayed?.sequence === meta.sequence && !preview.hidden) return;
    const next = new Image();
    next.id = 'd405-preview'; next.alt = preview.alt;
    await new Promise((resolve, reject) => {
      const finish = error => {
        clearTimeout(timer); next.onload = null; next.onerror = null;
        error ? reject(error) : resolve();
      };
      const timer = setTimeout(() => finish(new Error('画面读取超时')), 1800);
      next.onload = () => finish(); next.onerror = () => finish(new Error('画面读取失败'));
      next.src = path;
    });
    if (stopped || document.hidden || !frameFresh(meta, serverNow(clock))) return cameraUnavailable();
    preview.replaceWith(next); preview = next; displayed = meta;
    preview.hidden = false; byId('d405-message').hidden = true;
    byId('d405-status').textContent = '彩色实时';
    byId('d405-status').classList.add('online');
    byId('d405-detail').textContent = next.naturalWidth + ' × ' + next.naturalHeight + ' · 更新 ' + new Date(meta.captured_at * 1000).toLocaleTimeString('zh-CN', {hour12: false});
    byId('d405-depth').textContent = meta.depth?.state === 'disabled' ? '深度已关闭 · 本页仅使用彩色预览' : '本页未切换深度，请在控制台查看深度状态';
  } catch { cameraUnavailable(); }
}

function renderArm() {
  const label = side.value === 'left' ? '左臂' : '右臂';
  const arm = snapshot?.arms?.[side.value];
  byId('handeye-joints').textContent = '—';
  if (!['left', 'right'].includes(side.value)) {
    byId('arm-feedback-status').textContent = '请选择 D405 安装侧';
    return;
  }
  const age = serverNow(stateClock) - arm?.timestamp * 1000;
  if (snapshot?.ros?.connected !== true || arm?.fresh !== true ||
      !Number.isFinite(age) || age < -1500 || age > 2500 ||
      !Number.isFinite(arm?.age_s) || arm.age_s > 1.5 || arm.age_s < 0 ||
      !Array.isArray(arm?.joints) || arm.joints.length !== 6 || !arm.joints.every(Number.isFinite)) {
    byId('arm-feedback-status').textContent = label + '反馈缺失或已过期';
    return;
  }
  byId('arm-feedback-status').textContent = label + '反馈实时';
  byId('handeye-joints').textContent = arm.joints.map((value, index) => 'J' + (index + 1) + ' ' + (value * 180 / Math.PI).toFixed(1) + '°').join(' / ');
}

async function updateArm() {
  if (!side.value) return renderArm();
  try {
    const {data, clock} = await readJSON('/api/state');
    snapshot = data; stateClock = clock;
  } catch { snapshot = null; }
  renderArm();
}

side.addEventListener('change', () => { snapshot = null; renderArm(); });
// Preparation only: no capture or motion handlers are installed.
async function poll(task, delay) {
  while (!stopped) {
    if (!document.hidden) await task();
    await wait(delay);
  }
}
const watchdog = setInterval(() => {
  if (displayed && !frameFresh(displayed, serverNow(cameraClock))) cameraUnavailable();
  renderArm();
}, 250);
window.addEventListener('pagehide', () => { stopped = true; clearInterval(watchdog); });
window.addEventListener('pageshow', event => { if (event.persisted) window.location.reload(); });
poll(updateCamera, 400);
poll(updateArm, 800);
