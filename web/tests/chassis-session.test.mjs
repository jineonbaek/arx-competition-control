import test from 'node:test';
import assert from 'node:assert/strict';
import { webcrypto } from 'node:crypto';
import { readFileSync } from 'node:fs';
import vm from 'node:vm';

const source=readFileSync(new URL('../public/js/main.js',import.meta.url),'utf8');
const pageCode=source.slice(source.indexOf('const $'),source.indexOf('function toast'))+
  source.slice(source.indexOf('async function command('),source.indexOf('function restoreGroup('));

test('each page sends its own stable control ID on moves and brakes only',async()=>{
  const pages=[];
  for(let i=0;i<3;i++){
    const requests=[];
    const context=vm.createContext({crypto:webcrypto,newEditor:()=>({}),
      fetch:async(url,options)=>{
        requests.push(JSON.parse(options.body));
        return {ok:true,json:async()=>({ok:true})};
      }});
    vm.runInContext(pageCode,context);
    await vm.runInContext(`command({type:'chassis',x:.05,y:0,omega:0,sequence:++baseSequence},true)`,context);
    await vm.runInContext(`command({type:'chassis.stop',sequence:++baseSequence},true)`,context);
    await vm.runInContext(`command({type:'arm.stop',arm:'right'},true)`,context);
    assert.match(requests[0].control_id,/^[0-9a-f]{32}$/);
    assert.equal(requests[0].control_id,requests[1].control_id);
    assert.deepEqual(requests.map(r=>r.sequence),[1,2,undefined]);
    assert.equal(Object.hasOwn(requests[2],'control_id'),false);
    pages.push(requests[0].control_id);
  }
  assert.equal(new Set(pages).size,3);
});
