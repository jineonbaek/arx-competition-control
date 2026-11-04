import test from 'node:test';
import assert from 'node:assert/strict';
import { newEditor, followFeedback, checkedNumber, CHASSIS_KEYS, chassisKeyIntent } from '../public/js/state.js';
const feedback={fresh:true,joints:[Math.PI/2,0,0,0,0,0],gripper:-1,tcp:[.1,.2,.3,Math.PI,0,0]};
test('feedback maps units and missing data is not zero',()=>{
  const e=newEditor('left',feedback);
  assert.equal(e.joints[0],90);assert.equal(e.tcp[0],100);assert.equal(e.tcp[3],180);
  assert.equal(newEditor('right',{fresh:false}).joints,null);
});
test('manual drafts survive feedback',()=>{
  const e=newEditor('left',feedback);e.dirty.joints=true;e.joints[0]=12;
  followFeedback(e,{...feedback,joints:[0,0,0,0,0,0],gripper:-2});
  assert.equal(e.joints[0],12);assert.equal(e.gripper,-2);
});
test('arm switching discards drafts',()=>{
  const e=newEditor('left',feedback);e.dirty.joints=true;e.joints[0]=12;
  const next=newEditor('right',{...feedback,joints:[0,0,0,0,0,0]});
  assert.equal(next.joints[0],0);assert.equal(next.dirty.joints,false);
});
test('invalid input is rejected',()=>{
  for(const value of ['', ' ',NaN,Infinity,true,null,'bad',11])
    assert.equal(checkedNumber(value,-10,10).valid,false);
  assert.equal(checkedNumber('-2.5',-10,10).value,-2.5);
});

test('chassis keyboard mapping keeps six vendor directions',()=>{
  const expected={KeyW:'forward',KeyS:'backward',KeyA:'left',KeyD:'right',KeyQ:'yaw-left',KeyE:'yaw-right'};
  for(const [code,direction] of Object.entries(expected)){
    assert.equal(CHASSIS_KEYS[code].direction,direction);
    assert.deepEqual(chassisKeyIntent(new Set([code])),{kind:'move',code,...CHASSIS_KEYS[code]});
  }
});
test('space wins and multiple direction keys cannot become a combined command',()=>{
  assert.deepEqual(chassisKeyIntent(new Set(['KeyW','Space'])),{kind:'brake'});
  assert.deepEqual(chassisKeyIntent(new Set(['KeyW','KeyD'])),{kind:'conflict'});
  assert.deepEqual(chassisKeyIntent(new Set()),{kind:'idle'});
  assert.deepEqual(chassisKeyIntent(new Set(['ArrowUp'])),{kind:'idle'});
});

test('chassis input accepts positive values through each configured maximum',()=>{
  for(const max of [2,1]){
    for(const value of [.0005,.2,max]) assert.equal(checkedNumber(value,Number.MIN_VALUE,max).valid,true);
    for(const value of ['',0,-.1,max+.0001,NaN,Infinity]) assert.equal(checkedNumber(value,Number.MIN_VALUE,max).valid,false);
  }
});
