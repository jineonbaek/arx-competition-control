import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import {webcrypto} from 'node:crypto';
import {readFileSync} from 'node:fs';
const source=readFileSync(new URL('../public/js/map.js',import.meta.url),'utf8');
test('T2 page displays actual navigation pair and starts only on explicit click',async()=>{
 const nodes=new Map(),sent=[];
 const el=id=>{if(!nodes.has(id))nodes.set(id,{disabled:false,checked:true,textContent:'',getContext:()=>({}),addEventListener(){},setAttribute(){},replaceChildren(){}});return nodes.get(id);};
 const c=vm.createContext({crypto:webcrypto,AbortSignal,document:{body:{dataset:{taskKey:'t2_test',taskEndpoint:'/api/t2-test'}},getElementById:el,createElement:()=>el(Symbol())},fetch:async(url,options)=>{sent.push({url,data:JSON.parse(options.body)});return {ok:true,json:async()=>({status:'running',tasks:[],can_continue:false,command_epoch:17})};}});
 vm.runInContext(source.slice(0,source.indexOf('function draw()')),c);
 assert.equal(sent.length,0);
 vm.runInContext("directConnected=true;directTask={status:'idle',command_epoch:16,can_start:true,tasks:[],target:'第1点 → 第2点（到T2后放置）'};renderCompetition(directTask)",c);
 assert.equal(el('target').textContent,'第1点 → 第2点（到T2后放置）');
 await vm.runInContext("competitionAction('start')",c);
 assert.equal(sent[0].url,'/api/t2-test');assert.equal(sent[0].data.expected_epoch,16);assert.equal(sent[0].data.environment_clear,true);
});
