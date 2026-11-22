import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
const source=readFileSync(new URL('../public/js/main.js',import.meta.url),'utf8');
const block=source.slice(source.indexOf('async function fastPregrasp('),source.indexOf("$('right-place-preobserve').addEventListener"));
test('record saves actual backend pose; send reloads shared target and uses existing command',async()=>{
  const handlers={},commands=[],requests=[];
  let point=[19,29,43,-11,19,-88];
  const ctx={rightAvoidanceBusy:false,$:id=>({addEventListener:(_,fn)=>{handlers[id]=fn;}}),
    render:()=>{},toast:()=>{},localLog:()=>{},fmt:v=>String(v),
    crypto:{getRandomValues:a=>a},Uint8Array,
    fetch:async(url,options)=>{
      requests.push({url,options});
      return {ok:true,json:async()=>({settings:{fast_pregrasp_joints_deg:point}})};
    },command:async c=>commands.push(c)};
  vm.runInNewContext(block,ctx);
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(commands.length,0);
  await handlers['record-fast-pregrasp']();
  const posted=JSON.parse(requests.at(-1).options.body);
  assert.deepEqual(Object.keys(posted).sort(),['action','request_id']);
  assert.equal(posted.action,'record_fast_pregrasp');
  assert.equal(commands.length,0);
  point=[15,32,45,-13,16,-90];
  await handlers['right-fast-pregrasp']();
  assert.deepEqual(JSON.parse(JSON.stringify(commands)),[{type:'arm.joints',arm:'right',joints:point}]);
  ctx.fetch=async()=>({ok:false,json:async()=>({message:'unavailable'})});
  await handlers['right-fast-pregrasp']();
  assert.equal(commands.length,1);
});
