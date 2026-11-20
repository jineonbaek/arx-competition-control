import test from 'node:test';
import assert from 'node:assert/strict';

test('navigation debug reads only map state and never starts a robot task',async()=>{
 const previous={document:globalThis.document,fetch:globalThis.fetch,setTimeout:globalThis.setTimeout};
 const nodes=new Map(),calls=[],timers=[];
 const context={createImageData:(w,h)=>({data:new Uint8ClampedArray(w*h*4)}),
  clearRect(){},putImageData(){},save(){},restore(){},translate(){},rotate(){},beginPath(){},
  lineTo(){},moveTo(){},stroke(){},closePath(){},fill(){}};
 const node=id=>{if(!nodes.has(id))nodes.set(id,{textContent:'',hidden:false,getContext:()=>context});return nodes.get(id);};
 let data={map:null,pose:null,route:{available:false},error:'等待导航'};
 globalThis.document={getElementById:node};globalThis.setTimeout=(fn,delay)=>timers.push([fn,delay]);
 globalThis.fetch=async(path,options)=>{calls.push([path,options]);return {ok:true,json:async()=>data};};
 try{
  await import('../public/js/navigation-debug.js?test='+Date.now());await new Promise(resolve=>setImmediate(resolve));
  assert.match(node('nav-pose').textContent,/尚未有效/);
  data={map:{width:2,height:2,cells:[0,100,-1,0],origin:[0,0,0],resolution:.1,frame:'map'},
    pose:{valid:true,x:.1,y:.1,yaw:0,frame:'map'},route:{available:true,status:'待开始'}};
  await timers[0][0]();
  assert.match(node('nav-pose').textContent,/0\.100/);
  assert.equal(node('nav-route').textContent,'待开始');
  assert.equal(node('nav-map').width,2);
  assert.ok(calls.every(([path,options])=>path==='/api/navigation-view'&&(!options.method||options.method==='GET')));
 }finally{Object.assign(globalThis,previous);}
});
