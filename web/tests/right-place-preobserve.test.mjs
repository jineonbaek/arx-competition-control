import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const source=readFileSync(new URL('../public/js/main.js',import.meta.url),'utf8');
const target=source.match(/const rightPlacePreobserveJoints = .*;/)[0];
const start=source.indexOf('  const rightPlaceError=');
const status=source.slice(start,source.indexOf("  $('tcp-mode').textContent=",start));

test('right placement preset reuses server tolerance and fresh six-axis feedback',()=>{
  const el={};
  const snapshot={arms:{right:{fresh:true,joints:[0,0,25,-25,0,-90].map(v=>v*Math.PI/180)}}};
  function text(tolerance=1){
    vm.runInNewContext(target+status,{snapshot,rightAvoidance:{tolerance_deg:tolerance},$:()=>el,fmt:v=>v.toFixed(2)});
    return el.textContent;
  }
  assert.match(text(),/已到位/);
  snapshot.arms.right.joints[0]=.99*Math.PI/180;
  assert.match(text(),/已到位/);
  snapshot.arms.right.joints[0]=1.01*Math.PI/180;
  assert.doesNotMatch(text(),/已到位/);
  assert.match(text(2),/已到位/);
  snapshot.arms.right.fresh=false;
  assert.match(text(),/等待反馈/);
});

test('button sends only existing joint command when clicked',async()=>{
  const marker="$('right-place-preobserve').addEventListener('click',async()=>{";
  const handler=source.slice(source.indexOf(marker),source.indexOf("$('tcp-send').addEventListener",source.indexOf(marker)));
  let click;
  const commands=[];
  vm.runInNewContext(target+'let rightAvoidanceBusy=false;'+handler,{
    $:()=>({addEventListener:(_,fn)=>{click=fn;}}),render:()=>{},command:async c=>commands.push(c)
  });
  assert.equal(commands.length,0);
  await click();
  assert.deepEqual(JSON.parse(JSON.stringify(commands)),[{type:'arm.joints',arm:'right',joints:[0,0,25,-25,0,-90]}]);
});
