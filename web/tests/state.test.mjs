import test from 'node:test';
import assert from 'node:assert/strict';
import { newEditor, followFeedback, checkedNumber, chassisKeyIntent } from '../public/js/state.js';
const feedback={fresh:true,joints:[Math.PI/2,0,0,0,0,0],gripper:-1,tcp:[.9,.8,.7,0,0,0],base_wrist:[100,200,300,180,0,0]};
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

test('chassis keyboard maps each key to its signed axis',()=>{
  const expected={KeyW:[1,0,0],KeyS:[-1,0,0],KeyA:[0,1,0],KeyD:[0,-1,0],KeyQ:[0,0,1],KeyE:[0,0,-1]};
  for(const [code,axes] of Object.entries(expected)){
    const {kind,x,y,omega,codes}=chassisKeyIntent(new Set([code]));
    assert.equal(kind,'move');
    assert.deepEqual([x,y,omega],axes);
    assert.deepEqual(codes,[code]);
  }
});
test('independent axes combine and releasing a key updates the remaining axes',()=>{
  const keys=new Set(['KeyW','KeyA','KeyQ']);
  for(const [released,axes] of [[null,[1,1,1]],['KeyA',[1,0,1]],['KeyQ',[1,0,0]]]){
    keys.delete(released);
    const {kind,x,y,omega}=chassisKeyIntent(keys);
    assert.equal(kind,'move');
    assert.deepEqual([x,y,omega],axes);
  }
});
test('space wins and opposed keys still request a stop',()=>{
  assert.deepEqual(chassisKeyIntent(new Set(['KeyW','KeyA','KeyQ','Space'])),{kind:'brake'});
  for(const codes of [['KeyW','KeyS'],['KeyA','KeyD'],['KeyQ','KeyE']])
    assert.deepEqual(chassisKeyIntent(new Set(codes)),{kind:'conflict'});
  assert.deepEqual(chassisKeyIntent(new Set()),{kind:'idle'});
  assert.deepEqual(chassisKeyIntent(new Set(['ArrowUp'])),{kind:'idle'});
});

test('chassis input accepts positive values through each configured maximum',()=>{
  for(const max of [2,1]){
    for(const value of [.0005,.2,max]) assert.equal(checkedNumber(value,Number.MIN_VALUE,max).valid,true);
    for(const value of ['',0,-.1,max+.0001,NaN,Infinity]) assert.equal(checkedNumber(value,Number.MIN_VALUE,max).valid,false);
  }
});
