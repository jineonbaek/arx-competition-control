import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';

const html = fs.readFileSync(new URL('../public/visual-docking.html', import.meta.url), 'utf8');
const script = fs.readFileSync(new URL('../public/visual-docking.js', import.meta.url), 'utf8');

test('page exposes separate locate and place controls with D435 and D405 evidence', () => {
  for (const id of ['d435', 'd435-depth', 'd405-horizontal', 'd405-status', 'd405-metrics'])
    assert.match(html, new RegExp(`id="${id}"`));
  assert.equal((html.match(/id="start"/g) || []).length, 1);
  assert.match(html,/id="locate-only"/);assert.match(html,/id="debug-confirm"/);
  assert.doesNotMatch((html + script).toLowerCase(), /astra/);
});

test('live module refreshes D405 identity and renders correction metrics and timing', async () => {
  const previous = {document: globalThis.document, fetch: globalThis.fetch, setTimeout: globalThis.setTimeout};
  const nodes = new Map(); const calls = []; const timers = [];
  const node = id => {
    if (!nodes.has(id)) nodes.set(id, {id, textContent:'', hidden:false, disabled:false, src:'',
      removeAttribute(name){ if (name === 'src') this.src=''; }, replaceChildren(){this.children=[];},
      append(value){(this.children ||= []).push(value);}});
    return nodes.get(id);
  };
  let d405Sequence = 'frame-1',overlayMissing=false,rawSequence='raw-1',overlayAge=0;
  globalThis.document = {getElementById: node, createElement(tag){return node(`${tag}-${Math.random()}`);}};
  globalThis.setTimeout = (fn, delay) => {timers.push([fn, delay]); return timers.length;};
  globalThis.fetch = async (path, options) => {
    calls.push([path, options]);
    let value = {};
    if (path === '/api/fast-level') value = {status:'running', phase:'d405_horizontal_trim',
      d405_attempt:2, d405_raw_error_mm:-1.25, d405_step_mm:-1.25,
      d405_cumulative_correction_mm:-3.25, d405_converged:false, d405_degraded:true,
      d405_reason:'depth unavailable', timeline:[{phase:'d405_horizontal_trim',planned_duration_s:32,duration_s:5,cumulative_duration_s:9,status:'ok'}]};
    else if (path === '/camera-overlay/d435.json') value = {sequence:'d435-7', stale:false,updated_at:Date.now()/1000};
    else if (path === '/camera-overlay/d405-horizontal.json') {
      if(overlayMissing)return {ok:false,json:async()=>({message:'页面不存在'})};
      value = {status:'ready',frame_id:d405Sequence,image_time:Date.now()/1000-overlayAge};
    }
    else if (path === '/camera-live/d405.json') value={state:'online',image_file:rawSequence+'.jpg',captured_at:Date.now()/1000};
    else if (path === '/api/placement-reference') value = {};
    else if (path === '/api/placement-debug') value = {status:'idle',can_start:true,command_epoch:42,running:false};
    return {ok:true, json:async()=>value};
  };
  try {
    await import('../public/visual-docking.js?test=' + Date.now());
    await new Promise(resolve => setImmediate(resolve));
    assert.match(node('d435').src, /d435-7/);
    assert.match(node('d405-horizontal').src, /frame-1/);
    assert.match(node('d405-metrics').textContent, /-1\.25 mm.*-3\.25 mm.*第 2 次/);
    assert.match(node('d405-status').textContent, /降级.*depth unavailable/);
    assert.ok((node('timings').children || []).length > 0);
    d405Sequence = 'frame-2';
    const cameraTimer = timers.find(([,delay]) => delay === 900);
    assert.ok(cameraTimer); await cameraTimer[0](); await new Promise(resolve => setImmediate(resolve));
    assert.match(node('d405-horizontal').src, /frame-2/);
    assert.ok(calls.some(([path]) => path === '/camera-overlay/d405-horizontal.json'));
    assert.ok(calls.every(([path]) => !String(path).toLowerCase().includes('astra')));
    assert.equal(node('start').disabled, true);
    assert.ok(calls.every(([,options])=>!options?.method||options.method==='GET'));
    assert.ok(calls.every(([path])=>path!=='/api/placement-reference'));
    assert.equal(node('locate-only').disabled,false);
    await node('cancel').onclick();
    assert.equal(calls.filter(([,options])=>options?.method==='POST').length,0);
    await node('locate-only').onclick();
    let posts=calls.filter(([,options])=>options?.method==='POST');
    assert.equal(posts.length,1);assert.equal(posts[0][0],'/api/placement-debug');
    assert.equal(JSON.parse(posts[0][1].body).op,'observe');
    node('debug-confirm').checked=true;node('debug-confirm').onchange();
    assert.equal(node('start').disabled,false);
    await node('start').onclick();
    posts=calls.filter(([,options])=>options?.method==='POST');
    assert.equal(posts.length,2);
    const payload=JSON.parse(posts[1][1].body);
    assert.equal(payload.op,'place');assert.equal(payload.expected_epoch,42);
    assert.equal(payload.base_fixed_confirmed,true);
    await node('cancel').onclick();
    posts=calls.filter(([,options])=>options?.method==='POST');
    assert.equal(posts.length,3);assert.equal(posts[2][0],'/api/placement-debug');
    assert.equal(JSON.parse(posts[2][1].body).request_id,payload.request_id);
    assert.equal(JSON.parse(posts[2][1].body).op,'cancel');
    assert.ok(posts.every(([path])=>path!=='/api/fast-level'));
    assert.ok(calls.every(([path])=>!path.includes('competition-task')));
    overlayAge=30;
    await cameraTimer[0]();await new Promise(resolve=>setImmediate(resolve));
    assert.equal(node('d405-evidence').hidden,false);
    assert.match(node('d405-evidence').src,/frame-2/);
    assert.match(node('d405-evidence-status').textContent,/快照.*30.*实时/);
    assert.match(node('d405-horizontal').src,/camera-live/);
    overlayMissing=true;
    await cameraTimer[0]();await new Promise(resolve=>setImmediate(resolve));
    assert.equal(node('d405-horizontal').hidden,false);
    assert.match(node('d405-horizontal').src,/camera-live\/raw-1\.jpg/);
    assert.match(node('d405-camera-status').textContent,/实时彩色.*未生成/);
    rawSequence='raw-2';
    await cameraTimer[0]();await new Promise(resolve=>setImmediate(resolve));
    assert.match(node('d405-horizontal').src,/camera-live\/raw-2\.jpg/);
    assert.ok(calls.filter(([,options])=>options?.method==='POST').length===3);
    await node('refresh-d405').onclick();
    posts=calls.filter(([,options])=>options?.method==='POST');
    assert.equal(posts.length,4);assert.equal(posts[3][0],'/api/d405-placement-preview');
    assert.deepEqual(JSON.parse(posts[3][1].body),{op:'refresh'});
  } finally { Object.assign(globalThis, previous); }
});
