import {previewState, applyPreviewState} from './robot-preview-state.js';

const element = id => document.getElementById(id);
const text = (id, value) => { if (element(id).textContent !== value) element(id).textContent = value; };
let robot, renderer, controls, snapshot = null, stateClock = null, stopped = false, request;
const wait = ms => new Promise(resolve => setTimeout(resolve, ms));

function updateFeedback() {
  const now = stateClock ? stateClock.server + (performance.now() - stateClock.local) / 1000 : NaN;
  const value = previewState(snapshot, now);
  for (const [side, label] of [['left', '左臂'], ['right', '右臂']]) {
    const part = value[side];
    text('preview-' + side + '-status', label + (part.valid ? ' · 实时' : ' · 反馈缺失或过期'));
    element('preview-' + side + '-status').classList.toggle('online', part.valid);
    text('preview-' + side + '-joints', part.valid
      ? part.joints.map((q, i) => 'J' + (i + 1) + ' ' + (q * 180 / Math.PI).toFixed(1) + '°').join(' / ') : '—');
  }
  text('preview-lift-status', value.lift.valid ? '升降 · 实时（名义换算）' : '升降 · 反馈缺失或过期');
  element('preview-lift-status').classList.toggle('online', value.lift.valid);
  text('preview-lift-value', value.lift.valid
    ? value.lift.rad.toFixed(3) + ' rad → 约 ' + (value.lift.positionM * 1000).toFixed(1) + ' mm 行程' : '—');
  if (robot) {
    if(proposal && value.right.valid && value.lift.valid){
      const t=Math.min(1,(performance.now()-playback)/2500);
      value.right.joints=proposal.start.map((v,i)=>v+(proposal.target[i]-v)*t);
    }
    applyPreviewState(robot,value);
  }
}

let proposal=null, playback=0;
if(element('step-title')) window.addEventListener('handeye-preview',event=>{
  if(proposal?.ticket!==event.detail?.ticket)playback=performance.now();
  proposal=event.detail;
});
async function poll() {
  while (!stopped) {
    if (!document.hidden) {
      request = new AbortController();
      const timeout = setTimeout(() => request.abort(), 1500);
      try {
        const response = await fetch('/api/state', {cache: 'no-store', signal: request.signal});
        if (!response.ok) throw new Error('反馈读取失败');
        const data = await response.json();
        const serverDate = Date.parse(response.headers.get('Date')) / 1000;
        if (!Number.isFinite(serverDate) || Math.abs(data.time - serverDate) > 2) throw new Error('Invalid server clock');
        if (!Number.isFinite(data.time)) throw new Error('反馈时间无效');
        if (!stopped && !document.hidden) {
          snapshot = data;
          stateClock = {server: Math.max(serverDate, data.time), local: performance.now()};
        }
      } catch { snapshot = null; }
      finally { clearTimeout(timeout); }
    }
    updateFeedback();
    await wait(200);
  }
}

async function start() {
  try {
    const base = '../vendor/lift2-preview/';
    const [THREE, {OrbitControls}, {STLLoader}] = await Promise.all([
      import(base + 'three.module.js'), import(base + 'OrbitControls.js'), import(base + 'STLLoader.js')
    ]);
    globalThis.THREE = {...THREE, STLLoader};
    await import(base + 'URDFLoader.js');
    if (stopped) return;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x090f1a);
    const camera = new THREE.PerspectiveCamera(42, 1, .01, 30);
    const viewport = element('robot-preview-viewport');
    renderer = new THREE.WebGLRenderer({antialias: true});
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.domElement.setAttribute('aria-label', 'LIFT2 整机只读三维预览');
    viewport.append(renderer.domElement);
    renderer.domElement.addEventListener('webglcontextlost', event => {
      event.preventDefault();
      text('preview-model-status', '三维显示已中断，请刷新页面');
      element('preview-model-status').classList.add('preview-error');
      renderer.setAnimationLoop(null);
    });
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.enablePan = true;
    controls.minDistance = .3;
    controls.maxDistance = 8;
    scene.add(new THREE.HemisphereLight(0xddecff, 0x56647b, .7));
    for (const [x, y, z, intensity] of [[2, 3, 2, 1.2], [-2, 1, -1, .5]]) {
      const light = new THREE.DirectionalLight(0xffffff, intensity);
      light.position.set(x, y, z); scene.add(light);
    }
    scene.add(new THREE.GridHelper(3, 30, 0x2b5267, 0x19293b));
    const manager = new THREE.LoadingManager();
    const geometryReady = new Promise((resolve, reject) => {
      manager.onLoad = resolve;
      manager.onError = url => reject(new Error('模型零件加载失败：' + url));
    });
    const loader = new globalThis.URDFLoader(manager);
    loader.packages = {lift2: '/models/lift2/diy-20261105'};
    loader.parseCollision = false;
    const loaded = await Promise.all([loader.loadAsync('/models/lift2/diy-20261105/urdf/lift2_with_sensors.urdf'), geometryReady]);
    if (stopped) return;
    robot = loaded[0];
    for (const joint of Object.values(robot.joints)) joint.ignoreLimits = true;
    // Same URDF-to-Three convention as TH; CAD wheel contact is 0.168 m below base origin.
    robot.rotation.x = -Math.PI / 2;
    robot.position.y = .168;
    scene.add(robot);
    const box = new THREE.Box3().setFromObject(robot);
    const sphere = box.getBoundingSphere(new THREE.Sphere());
    const fit = () => {
      const horizontal = 2 * Math.atan(Math.tan(camera.fov * Math.PI / 360) * camera.aspect);
      const distance = sphere.radius / Math.sin(Math.min(horizontal, camera.fov * Math.PI / 180) / 2) * 1.15;
      camera.position.copy(sphere.center).add(new THREE.Vector3(1.3, .75, 1.6).normalize().multiplyScalar(distance));
      controls.target.copy(sphere.center); controls.update();
    };
    const resize = () => {
      const width = viewport.clientWidth, height = viewport.clientHeight;
      if (!width || !height) return;
      camera.aspect = width / height; camera.updateProjectionMatrix();
      renderer.setSize(width, height, false);
    };
    const observer = new ResizeObserver(resize);
    observer.observe(viewport);
    resize(); fit(); updateFeedback();
    element('preview-reset-view').disabled = false;
    element('preview-reset-view').addEventListener('click', fit);
    text('preview-model-status',element('step-title') ? '右臂路径模拟 · 立柱检查结果见逐点采集' : 'LIFT2S DIY 模型已加载 · 仅预览，无碰撞保护');
    element('robot-preview').dataset.loaded = 'true';
    renderer.setAnimationLoop(() => {
      updateFeedback(); controls.update(); renderer.render(scene, camera);
    });
    window.addEventListener('pagehide', () => observer.disconnect(), {once: true});
  } catch (error) {
    text('preview-model-status', '模型加载失败：' + error.message);
    element('preview-model-status').classList.add('preview-error');
    renderer?.setAnimationLoop(null);
  }
}

document.addEventListener('visibilitychange', () => {
  snapshot = null; request?.abort(); updateFeedback();
});
window.addEventListener('pagehide', () => {
  stopped = true; request?.abort(); renderer?.setAnimationLoop(null);
  controls?.dispose(); renderer?.dispose();
});
window.addEventListener('pageshow', event => { if (event.persisted) window.location.reload(); });
poll();
start();
