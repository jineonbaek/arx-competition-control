import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';

test('placement page exposes three independent stages, editable point and truthful height sources',()=>{
  const html=fs.readFileSync(new URL('../public/visual-docking.html',import.meta.url),'utf8');
  for(const id of ['stage-preobserve','stage-correct','stage-insert','stage-record-point','stage-save','stage-z','stage-model-z','stage-target-z','stage-cabinet-z','stage-z-plus-1','stage-z-minus-1','stage-xyz-x','stage-joint-6'])assert.match(html,new RegExp(`id="${id}"`));
  const correction=html.match(/<article id="d405-correction">([\s\S]*?)<\/article>/)[1];
  for(const id of ['stage-correct','refresh-d405','d405-evidence','stage-y-result'])assert.match(correction,new RegExp('id="'+id+'"'));
  assert.match(html,/630/);assert.match(html,/暂定/);assert.match(html,/placement-stages.js/);
});

test('stage controls only post on user action and polling preserves edits',async()=>{
  const previous={document:globalThis.document,fetch:globalThis.fetch,setTimeout:globalThis.setTimeout};
  const nodes=new Map(),calls=[],timers=[];
  let failure=null,pending=null;
  const node=id=>{if(!nodes.has(id))nodes.set(id,{id,value:'',checked:false,disabled:false,textContent:'',hidden:false,dataset:{},
    replaceChildren(){this.children=[];},append(v){(this.children||=[]).push(v);},setAttribute(){}});return nodes.get(id);};
  const settings={preobserve_joints_deg:[1,2,3,4,5,-90],preobserve_source:'saved',preobserve_lift_rad:0,
    cabinet_z_mm:630,cabinet_height_provisional:true,target_z_mm:630,z_reference:'visual',correction_axes:'z',
    y_target_offset_mm:0,correction_step_mm:3,tolerance_mm:1,insert_distance_mm:160,insert_speed_mm_s:8};
  let reply={settings,can_start:true,running:false,command_epoch:88,status:'idle',operation:null,
    height:{tcp_z_mm:629.2,model_tcp_z_mm:510,relative_cabinet_z_mm:-.8,visual_z_correction_mm:119.2,source:'d405_fixed_cabinet',age_s:.1},
    selected_height:{tcp_z_mm:629.2,source:'d405_fixed_cabinet'},preobserve_tcp_mm:[440,-160,500],preobserve_tool_rpy_deg:[0,0,0],timeline:[]};
  globalThis.document={getElementById:node,createElement:tag=>node(tag+Math.random())};
  globalThis.setTimeout=(fn,ms)=>{timers.push([fn,ms]);return timers.length;};
  globalThis.fetch=async(path,options)=>{
    calls.push([path,options]);
    if(options?.method==='POST'){
      const data=JSON.parse(options.body);
      if(pending)await pending;
      if(failure)return {ok:false,json:async()=>({message:failure})};
      if(data.op==='save_settings')reply={...reply,settings:data.settings};
      if(data.op==='run')reply={...reply,status:'running',running:true,can_start:false,request_id:data.request_id,operation:data.phase};
      if(data.op==='cancel')reply={...reply,status:'held',running:false,can_start:true};
    }
    return {ok:true,json:async()=>reply};
  };
  try{
    await import('../public/placement-stages.js?test='+Date.now());await new Promise(r=>setImmediate(r));
    assert.equal(calls.filter(([,o])=>o?.method==='POST').length,0);
    assert.match(node('stage-z').textContent,/629\.2/);assert.match(node('stage-model-z').textContent,/510\.0/);
    assert.equal(node('stage-preobserve').disabled,false);assert.equal(node('stage-correct').disabled,false);assert.equal(node('stage-insert').disabled,false);
    await node('stage-probe').onclick();
    const probe=calls.filter(([,o])=>o?.method==='POST').map(([,o])=>JSON.parse(o.body)).at(-1);
    assert.equal(probe.phase,'insert');assert.equal(probe.probe_only,true);
    const probePoll=timers.find(([,ms])=>ms===400)[0];
    reply={...reply,push:{cross_track_mm:2.67,peak_cross_track_mm:3.12}};
    await probePoll();
    assert.match(node('stage-travel').textContent,/偏离推进轴 2.7 mm.*峰值 3.1 mm/);
    reply={...reply,push:{mouth_then_push:true,rotation_drift_deg:23.4,measured_travel_mm:149.5,verified_depth_mm:150}};
    await probePoll();
    assert.match(node('stage-travel').textContent,/23.40°（仅记录）/);
    assert.match(node('stage-insertion-evidence').textContent,/149.5 mm.*至少 150 mm.*本体入仓深度 150.0 mm/);
    await node('stage-stop').onclick();calls.length=0;
    node('stage-target-z').value='635';node('stage-target-z').oninput?.();
    const poll=timers.find(([,ms])=>ms===400);assert.ok(poll);await poll[0]();
    assert.equal(node('stage-target-z').value,'635');
    node('stage-correct-axes').value='y';
    await node('stage-correct').onclick();
    let posts=calls.filter(([,o])=>o?.method==='POST').map(([,o])=>JSON.parse(o.body));
    assert.equal(posts[0].op,'save_settings');assert.equal(posts[0].settings.target_z_mm,635);
    assert.equal(posts[0].settings.correction_axes,'y');
    assert.equal(posts[1].phase,'correct');assert.equal(posts[1].expected_epoch,88);
    assert.equal(posts.filter(p=>p.op==='run').length,1);
    assert.equal(node('stage-feedback').dataset.level,'pending');
    reply={...reply,d405_attempt:2,d405_left_clearance_mm:8.7};
    await poll[0]();
    assert.match(node('stage-y-result').textContent,/第 2 次.*8.7 mm.*修正中/);
    reply={...reply,status:'held',running:false,can_start:true,reason:'轨迹未能下发'};
    await poll[0]();
    assert.equal(node('stage-feedback').dataset.level,'error');
    assert.match(node('stage-feedback').textContent,/轨迹未能下发/);
    // Start another run so the stop button targets an active request.
    await node('stage-correct').onclick();
    await node('stage-stop').onclick();
    posts=calls.filter(([,o])=>o?.method==='POST').map(([,o])=>JSON.parse(o.body));
    assert.deepEqual(posts.at(-1),{op:'cancel',request_id:posts.at(-2).request_id});
    await node('stage-z-plus-1').onclick();
    posts=calls.filter(([,o])=>o?.method==='POST').map(([,o])=>JSON.parse(o.body));
    assert.equal(posts.at(-1).phase,'z');assert.equal(posts.at(-1).delta_z_mm,1);
    assert.ok(calls.every(([path])=>path==='/api/placement-stages'));
    await node('stage-stop').onclick();
    let release;pending=new Promise(resolve=>{release=resolve;});failure='仓口有效深度不足';
    const action=node('stage-correct').onclick();
    assert.match(node('stage-feedback').textContent,/D405 修正.*正在提交/);
    release();await action;pending=null;
    assert.match(node('stage-feedback').textContent,/D405 修正.*未收到成功回执.*仓口有效深度不足/);
    await poll[0]();
    assert.match(node('stage-feedback').textContent,/仓口有效深度不足/,'polling must preserve the rejected action');
    reply={...reply,status:'completed',running:false,can_start:true,operation:'preobserve',reason:'已到预观测点；等待你选择下一步',
      timeline:[{phase:'preobserve',planned_duration_s:0,duration_s:.002,status:'ok'}],trajectory_handoffs:[]};
    await poll[0]();
    assert.match(node('stage-status').textContent,/当前位置已在预观测点.*无需移动/);
  }finally{Object.assign(globalThis,previous);}
});
