import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';

const html = fs.readFileSync(new URL('../public/visual-docking.html', import.meta.url), 'utf8');
const script = fs.readFileSync(new URL('../public/visual-docking.js', import.meta.url), 'utf8');

test('page contains independent D435 and D405 evidence without a second start control', () => {
  for (const id of ['d435', 'd435-depth', 'd405-horizontal', 'd405-status', 'd405-metrics'])
    assert.match(html, new RegExp(`id="${id}"`));
  assert.equal((html.match(/id="start"/g) || []).length, 1);
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
  let d405Sequence = 'frame-1';
  globalThis.document = {getElementById: node, createElement(tag){return node(`${tag}-${Math.random()}`);}};
  globalThis.setTimeout = (fn, delay) => {timers.push([fn, delay]); return timers.length;};
  globalThis.fetch = async (path, options) => {
    calls.push([path, options]);
    let value = {};
    if (path === '/api/fast-level') value = {status:'running', phase:'d405_horizontal_trim',
      d405_attempt:2, d405_raw_error_mm:-1.25, d405_step_mm:-1.25,
      d405_cumulative_correction_mm:-3.25, d405_converged:false, d405_degraded:true,
      d405_reason:'depth unavailable', timeline:[{phase:'d405_horizontal_trim',planned_duration_s:32,duration_s:5,cumulative_duration_s:9,status:'ok'}]};
    else if (path === '/camera-overlay/d435.json') value = {sequence:'d435-7', stale:false};
    else if (path === '/camera-overlay/d405-horizontal.json') value = {status:'ready',frame_id:d405Sequence,image_time:12};
    else if (path === '/api/placement-reference') value = {};
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
  } finally { Object.assign(globalThis, previous); }
});
