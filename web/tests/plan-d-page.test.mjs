import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import assert from 'node:assert/strict';

const source=readFileSync(new URL('../public/js/grasp-integration.js',import.meta.url),'utf8');
const controls=source.slice(source.indexOf('function executionButtons(){'),source.indexOf('const executionNames='));
test('PlanD preview stays separate while all three existing modes remain executable',()=>{
 const nodes={};
 const el=id=>nodes[id]??=(id==='environment-clear'?{checked:true}:{});
 const context=vm.createContext({el,execution:{status:'idle',available:true},planDBusy:false});
 vm.runInContext(controls,context);
 for(const mode of ['d405','d435','fixed']){
  el('vision-mode').value=mode;
  vm.runInContext('executionButtons()',context);
  assert.equal(el('execute').disabled,false,mode);
  assert.equal(el('plan-d-panel').hidden,true,mode);
 }
 el('vision-mode').value='plan_d';
 vm.runInContext('executionButtons()',context);
 assert.equal(el('execute').disabled,true);
 assert.equal(el('plan-d-panel').hidden,false);
 assert.equal(el('plan-d-capture').disabled,false);
 context.execution.status='running';
 vm.runInContext('executionButtons()',context);
 assert.equal(el('plan-d-capture').disabled,true);
});
