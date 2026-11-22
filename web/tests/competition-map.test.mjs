import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import {webcrypto} from 'node:crypto';
import {readFileSync} from 'node:fs';

const source=readFileSync(new URL('../public/js/map.js',import.meta.url),'utf8');
const code=source.slice(0,source.indexOf('function draw()'));
function page(){
  const nodes=new Map(),sent=[];
  function node(id){
    if(!nodes.has(id))nodes.set(id,{disabled:false,checked:true,textContent:'',children:[],
      getContext:()=>({}),setAttribute(){},addEventListener(){},replaceChildren(...children){this.children=children;}});
    return nodes.get(id);
  }
  const context=vm.createContext({crypto:webcrypto,AbortSignal,
    document:{getElementById:node,createElement:()=>node(Symbol())},
    fetch:async(url,options)=>{sent.push({url,data:JSON.parse(options.body)});return {ok:true,json:async()=>({status:'paused',can_continue:true,control_owned:false,tasks:[]})};}});
  vm.runInContext(code,context);
  return {context,node,sent};
}

test('map controls show missing capabilities and manual ownership honestly',()=>{
  const p=page();
  vm.runInContext(`connected=true;renderCompetition({status:'needs_attention',can_start:true,can_continue:true,control_owned:false,tasks:[{id:'navigate_pick',label:'到取物区',connected:false,status:'pending'}]})`,p.context);
  assert.equal(p.node('competition-start').disabled,false);
  assert.equal(p.node('competition-continue').disabled,false);
  assert.equal(p.node('competition-stop').disabled,true);
  assert.match(p.node('mission').textContent,/9983 手动/);
  assert.match(p.node('stages').children[0].textContent,/能力未接入/);
  p.node('environment-clear').checked=false;
  vm.runInContext(`renderCompetition({status:'recovering',control_owned:true,tasks:[]})`,p.context);
  assert.equal(p.node('competition-start').disabled,true);
  assert.equal(p.node('competition-stop').disabled,false);
  assert.match(p.node('task-state').textContent,/Qwen/);
});

test('start stop continue use task endpoint; stop remains callable while another request is busy',async()=>{
  const p=page();
  vm.runInContext(`connected=true;latest={competition:{command_epoch:42}}`,p.context);
  await vm.runInContext(`competitionAction('start')`,p.context);
  vm.runInContext(`taskBusy=true`,p.context);
  await vm.runInContext(`competitionAction('stop')`,p.context);
  vm.runInContext(`latest.competition.command_epoch=43`,p.context);
  await vm.runInContext(`competitionAction('continue')`,p.context);
  assert.deepEqual(p.sent.map(x=>x.data.action),['start','stop','continue']);
  assert.ok(p.sent.every(x=>x.url==='/api/competition-task'));
  assert.equal(p.sent[0].data.expected_epoch,42);
  assert.equal(p.sent[0].data.environment_clear,true);
  assert.equal(Object.hasOwn(p.sent[1].data,'expected_epoch'),false);
  assert.equal(new Set(p.sent.map(x=>x.data.request_id)).size,3);
});

test('fast selection is sent only for competition start and preserves continuation mode',async()=>{
  const p=page();
  vm.runInContext(`connected=true;latest={competition:{command_epoch:42}}`,p.context);
  p.node('competition-mode').value='fast';
  await vm.runInContext(`competitionAction('start')`,p.context);
  assert.equal(p.sent[0].data.mode,'fast');
  await vm.runInContext(`competitionAction('continue')`,p.context);
  assert.equal(Object.hasOwn(p.sent[1].data,'mode'),false);
  p.node('competition-mode').value='stable';
  await vm.runInContext(`competitionAction('start')`,p.context);
  assert.equal(Object.hasOwn(p.sent[2].data,'mode'),false);
});

test('navigation progress and handoff waits are displayed without adding motion controls',()=>{
  const p=page();
  vm.runInContext(`renderCompetition({status:'running',control_owned:true,tasks:[],navigation:{registered:true,
    route:{available:true,status:'执行中 1/5',drive_status:'waiting_for_replan'},
    handoff:{task:{point_id:'point_1',status:'resume_pending'},navigation_check:{reason:'等待实测停稳'}}}})`,p.context);
  assert.match(p.node('navigation-state').textContent,/执行中 1\/5/);
  assert.match(p.node('navigation-handoff').textContent,/point_1.*resume_pending.*等待实测停稳/);
  assert.equal(p.sent.length,0);
  vm.runInContext(`renderCompetition({tasks:[],navigation:{route:{available:false,status:'旧进度'},error:'导航离线'}})`,p.context);
  assert.match(p.node('navigation-state').textContent,/导航离线/);
});
