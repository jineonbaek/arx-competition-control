import test from 'node:test';
import assert from 'node:assert/strict';
import {existsSync, readFileSync} from 'node:fs';
const moduleURL = new URL('../public/js/robot-preview-state.js', import.meta.url);
async function api() {
  assert.ok(existsSync(moduleURL), 'Missing read-only feedback-to-model mapping');
  return import(moduleURL);
}
function state() {
  return {time:100, ros:{connected:true}, arms:{
    left:{fresh:true,timestamp:99.9,age_s:.1,joints:[.1,.2,.3,.4,.5,.6]},
    right:{fresh:true,timestamp:99.9,age_s:.1,joints:[-.7,.8,-.9,1,1.1,1.2]}
  },body:{fresh:true,timestamp:99.9,age_s:.1,height:8.308}};
}
test('maps both six-axis feedback arrays in radians and nominal lift displacement', async()=>{
  const {previewState}=await api();const value=previewState(state(),100);
  assert.deepEqual(value.left.joints,[.1,.2,.3,.4,.5,.6]);
  assert.deepEqual(value.right.joints,[-.7,.8,-.9,1,1.1,1.2]);
  assert.equal(value.lift.positionM,.2);
  assert.equal(value.lift.rad,8.308);
});
test('invalid or stale components fail independently and never become a zero pose',async()=>{
  const {previewState}=await api();const s=state();s.arms.left.fresh=false;s.body.height=NaN;
  const value=previewState(s,100);
  assert.equal(value.left.valid,false);assert.equal(value.left.joints,null);
  assert.equal(value.right.valid,true);assert.equal(value.lift.valid,false);
  assert.equal(value.lift.positionM,null);
  const expired=previewState(state(),103);
  assert.equal(expired.right.valid,false);assert.equal(expired.lift.valid,false);
});
test('rejects missing, malformed, future or disconnected feedback',async()=>{
  const {previewState}=await api();
  for(const change of [s=>s.ros.connected=false,s=>s.arms.right.joints=[0],s=>s.arms.right.joints[0]=Infinity,s=>s.arms.right.timestamp=105,s=>s.arms.right.age_s=-1,s=>s.arms.right.age_s=4]){
    const s=state();change(s);assert.equal(previewState(s,100).right.valid,false);
  }
  assert.equal(previewState(null,100).lift.valid,false);
});
test('does not clamp feedback to CAD/control limits or use target inputs',async()=>{
  const {previewState}=await api();const s=state();s.arms.right.joints[0]=-2.5;s.target={joints:[0,0,0,0,0,0]};s.body.height=20;
  assert.equal(previewState(s,100).right.joints[0],-2.5);
  assert.equal(previewState(s,100).lift.positionM,20/41.54);
});
test('updates actual URDF joint names and hides stale movable subtrees',async()=>{
  const {previewState,applyPreviewState}=await api();const calls={};
  const robot={joints:{},links:{body4:{visible:true},left_link11:{visible:true},right_link21:{visible:true}}};
  for(const side of ['left','right'])for(let i=0;i<6;i++){const name=side+'_joint'+((side==='left'?11:21)+i);robot.joints[name]={setJointValue:v=>calls[name]=v};}
  robot.joints.joint4={setJointValue:v=>calls.joint4=v};
  const s=state();applyPreviewState(robot,previewState(s,100));
  assert.equal(calls.right_joint21,-.7);assert.equal(calls.left_joint16,.6);assert.equal(calls.joint4,.2);
  s.arms.left.fresh=false;applyPreviewState(robot,previewState(s,100));
  assert.equal(robot.links.left_link11.visible,false);assert.equal(robot.links.right_link21.visible,true);
  applyPreviewState(robot,previewState(s,103));assert.equal(robot.links.body4.visible,false);
});
test('top page entry is independent of controls and preview has no command transport',async()=>{
  const html=readFileSync(new URL('../public/index.html',import.meta.url),'utf8');
  assert.ok(html.includes('id="robot-preview"'),'Missing top preview panel');
  assert.ok(html.indexOf('id="robot-preview"')<html.indexOf('class="dashboard"'));
  const viewer=readFileSync(new URL('../public/js/robot-preview.js',import.meta.url),'utf8');
  assert.ok(viewer.includes('/api/state'));
  assert.doesNotMatch(viewer,/\/api\/command|\/api\/camera\/depth|EventSource|WebSocket|main\.js|keydown|keyCode/);
});
