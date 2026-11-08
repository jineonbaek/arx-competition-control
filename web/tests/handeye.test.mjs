import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import {frameFresh, cameraImagePath} from '../public/js/camera-freshness.js';

function page({stale=false,offline=false}={}) {
  const nodes = new Map(), timers = [], intervals = [], events = new Map(), calls = [];
  let now = 100000, reloads = 0;
  class Element {
    constructor(id='') {
      this.id=id; this.value=''; this.textContent=''; this.hidden=id==='d405-preview';
      this.disabled=id==='handeye-start'; this.alt='D405';
      this.handlers=new Map(); this.classList={add(){},remove(){}};
    }
    addEventListener(name, callback) { this.handlers.set(name, callback); }
    dispatchEvent(event) { this.handlers.get(event.type)?.(event); }
    click() { if(!this.disabled)this.handlers.get('click')?.({type:'click'}); }
    replaceWith(next) { nodes.set(this.id,next); }
  }
  for(const id of ['handeye-arm','d405-preview','d405-message','d405-status',
    'd405-detail','d405-depth','arm-feedback-status','handeye-joints','handeye-start'])nodes.set(id,new Element(id));
  const arm={joints:[.1,.2,.3,.4,.5,.6],fresh:!stale,age_s:stale?30:.1,timestamp:stale?70:99.9};
  const meta={state:offline?'offline':'online',capture_session:'fixture',sequence:1,
    captured_at:stale?70:99.9,image_file:'d405-v-abc123-1.jpg',depth:{state:'disabled'}};
  const context={
    document:{hidden:false,getElementById:id=>nodes.get(id)},
    window:{addEventListener:(name,callback)=>events.set(name,callback),location:{reload(){reloads++;}}},
    performance:{now:()=>now}, AbortController, frameFresh, cameraImagePath,
    fetch:async(path,options={})=>{
      calls.push({path,method:options.method||'GET'});
      assert.ok(['/camera-live/d405.json','/api/state'].includes(path),'unexpected endpoint '+path);
      return {ok:true,headers:{get:()=>new Date(100000).toUTCString()},
        json:async()=>path==='/api/state'?{ros:{connected:true},arms:{left:arm,right:arm}}:meta};
    },
    Image:class extends Element {
      constructor(){super();this.naturalWidth=640;this.naturalHeight=480;}
      set src(value){this.source=value;queueMicrotask(()=>this.onload?.());}
    },
    setTimeout:(callback,delay)=>{const t={callback,delay,active:true};timers.push(t);return t;},
    clearTimeout:timer=>{timer.active=false;},
    setInterval:callback=>{const t={callback,active:true};intervals.push(t);return t;},
    clearInterval:timer=>{timer.active=false;}
  };
  const source=readFileSync(new URL('../public/js/handeye.js',import.meta.url),'utf8').replace(/^import[^\n]+\n/,'');
  vm.runInNewContext(source,context);
  async function settle(){for(let i=0;i<15;i++)await Promise.resolve();}
  return {nodes,events,calls,intervals,settle,get reloads(){return reloads;},
    async choose(side){
      nodes.get('handeye-arm').value=side;
      nodes.get('handeye-arm').dispatchEvent({type:'change'});
      const timer=timers.find(t=>t.active&&t.delay===800);
      assert.ok(timer);timer.active=false;timer.callback();await settle();
    },
    expire(){now+=5000;for(const t of intervals)if(t.active)t.callback();}
  };
}

function readOnly(h){
  assert.ok(h.calls.length>0);
  assert.ok(h.calls.every(c=>c.method==='GET'));
  assert.ok(h.nodes.get('handeye-start').disabled);
}

test('initial view reads fresh D405 color without choosing an arm or enabling depth',async()=>{
  const h=page();await h.settle();readOnly(h);
  assert.ok(h.calls.every(c=>c.path==='/camera-live/d405.json'));
  assert.equal(h.nodes.get('d405-preview').hidden,false);
  assert.match(h.nodes.get('d405-detail').textContent,/640 × 480/);
  assert.equal(h.nodes.get('handeye-arm').value,'');
  assert.match(h.nodes.get('arm-feedback-status').textContent,/请选择/);
});

test('selecting an arm shows actual joint angles but cannot start capture',async()=>{
  const h=page();await h.settle();await h.choose('right');readOnly(h);
  assert.equal(h.nodes.get('arm-feedback-status').textContent,'右臂反馈实时');
  assert.match(h.nodes.get('handeye-joints').textContent,/J1 5.7°/);
  const count=h.calls.length;h.nodes.get('handeye-start').click();
  assert.equal(h.calls.length,count);
  // Even changing disabled in devtools cannot invoke a handler that does not exist.
  h.nodes.get('handeye-start').disabled=false;h.nodes.get('handeye-start').click();
  assert.equal(h.calls.length,count);
});

test('stale metadata and stale joints do not appear as live feedback',async()=>{
  const h=page({stale:true});await h.settle();await h.choose('left');readOnly(h);
  assert.equal(h.nodes.get('d405-preview').hidden,true);
  assert.match(h.nodes.get('arm-feedback-status').textContent,/反馈缺失或已过期/);
  assert.equal(h.nodes.get('handeye-joints').textContent,'—');
});

test('an offline camera cannot show its previously available JPEG',async()=>{
  const h=page({offline:true});await h.settle();readOnly(h);
  assert.equal(h.nodes.get('d405-preview').hidden,true);
  assert.match(h.nodes.get('d405-message').textContent,/离线或已过期/);
});

test('watchdog clears frozen color and arm values even without another response',async()=>{
  const h=page();await h.settle();await h.choose('left');
  assert.equal(h.nodes.get('d405-preview').hidden,false);
  h.expire();readOnly(h);
  assert.equal(h.nodes.get('d405-preview').hidden,true);
  assert.equal(h.nodes.get('handeye-joints').textContent,'—');
});

test('BFCache restoration reloads and normal pageshow does not loop',async()=>{
  const h=page();await h.settle();
  h.events.get('pagehide')({persisted:true});
  assert.ok(h.intervals.every(t=>!t.active));
  h.events.get('pageshow')({persisted:true});
  assert.equal(h.reloads,1);
  h.events.get('pageshow')({persisted:false});
  assert.equal(h.reloads,1);
});
