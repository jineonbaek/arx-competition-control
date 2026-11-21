import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import {webcrypto} from 'node:crypto';
import {readFileSync} from 'node:fs';
const source=readFileSync(new URL('../public/js/map.js',import.meta.url),'utf8');
test('T2 page starts only the real T2 endpoint and leaves whole-competition state alone',async()=>{
 const nodes=new Map(),sent=[];
 const node=id=>{if(!nodes.has(id))nodes.set(id,{disabled:false,checked:true,textContent:'',getContext:()=>({}),setAttribute(){},addEventListener(){},replaceChildren(){}});return nodes.get(id);};
 const context=vm.createContext({crypto:webcrypto,AbortSignal,document:{body:{dataset:{taskKey:'t2_test',taskEndpoint:'/api/t2-test'}},getElementById:node,createElement:()=>node(Symbol())},fetch:async(url,options)=>{sent.push({url,data:JSON.parse(options.body)});return {ok:true,json:async()=>({status:'running',tasks:[],can_continue:false,command_epoch:12})};}});
 vm.runInContext(source.slice(0,source.indexOf('function draw()')),context);
 assert.equal(sent.length,0);
 vm.runInContext(`connected=false; latest={competition:{run_id:'whole'}};directTask={command_epoch:11,can_start:true,tasks:[]};directConnected=true;renderCompetition(directTask)`,context);
 assert.equal(node('competition-start').disabled,false);
 await vm.runInContext(`competitionAction('start')`,context);
 assert.equal(sent[0].url,'/api/t2-test');assert.equal(sent[0].data.expected_epoch,11);assert.equal(sent[0].data.environment_clear,true);
 assert.equal(vm.runInContext('latest.competition.run_id',context),'whole');assert.equal(node('competition-continue').disabled,true);
 await vm.runInContext(`competitionAction('stop')`,context);assert.equal(sent[1].data.action,'stop');assert.equal(sent[1].url,'/api/t2-test');
});
