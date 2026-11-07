import {test} from 'node:test';
import assert from 'node:assert/strict';
import {existsSync} from 'node:fs';
test('freshness rejects offline, frozen, malformed and future camera frames',async()=>{
 const url=new URL('../public/js/camera-freshness.js',import.meta.url);
 assert.ok(existsSync(url),'Camera freshness check not implemented');
 const {frameFresh}=await import(url);
 assert.equal(frameFresh({state:'online',captured_at:100},101000),true);
 assert.equal(frameFresh({state:'online',captured_at:100},104000),false);
 assert.equal(frameFresh({state:'offline',captured_at:100},101000),false);
 assert.equal(frameFresh({state:'online',captured_at:110},101000),false);
 assert.equal(frameFresh({state:'online',captured_at:null},101000),false);
 assert.equal(frameFresh({},101000),false);
 assert.equal(frameFresh({state:'online',captured_at:100},NaN),false);
});
test('vision frame URL uses immutable matching filename and rejects arbitrary paths',async()=>{
 const module=await import('../public/js/camera-freshness.js');
 assert.equal(typeof module.cameraImagePath,'function');
 assert.equal(module.cameraImagePath('d435',{image_file:'d435-v-abc123-42.jpg'}),'/camera-live/d435-v-abc123-42.jpg');
 assert.equal(module.cameraImagePath('d435',{image_file:'../config/control.json'}),'/camera-live/d435.jpg');
});
test('other camera box frames are matched and stale inference falls back to raw preview',async()=>{
 const m=await import('../public/js/camera-freshness.js');
 assert.equal(m.cameraImagePath('d405',{image_file:'d405-b-abc123-9.jpg'}),'/camera-live/d405-b-abc123-9.jpg');
 assert.equal(m.cameraImagePath('astra',{image_file:'astra-v-abc123-9.jpg'}),'/camera-live/astra-v-abc123-9.jpg');
 const raw={state:'online',captured_at:100};
 const boxed={state:'online',captured_at:100,source_camera:'d405',whole_battery:{boxes:[]}};
 assert.equal(m.chooseCameraFrame('d405',raw,boxed,101000),boxed);
 assert.equal(m.chooseCameraFrame('d405',raw,boxed,104000),raw);
 assert.equal(m.chooseCameraFrame('astra',raw,boxed,101000),raw);
});
test('all three cameras prefer a fresh joint body-and-hole frame',async()=>{
 const m=await import('../public/js/camera-freshness.js');
 for(const key of ['d435','d405','astra']){
  const raw={state:'online',captured_at:100};
  const combined={state:'online',captured_at:100,source_camera:key,whole_battery:{boxes:[],holes:[],hole_count:0}};
  assert.equal(m.chooseCameraFrame(key,raw,combined,101000),combined);
 }
});

test('depth button distinguishes requested enable, active stream and confirmed shutdown',async()=>{
 const {depthButtonView}=await import('../public/js/camera-freshness.js');
 assert.equal(depthButtonView().disabled,true);
 const off=depthButtonView({requested:false,state:'disabled',active:false,color_online:true});
 assert.equal(off.label,'开启深度');assert.equal(off.disabled,false);assert.equal(off.pressed,false);
 const pending=depthButtonView({requested:true,state:'starting',active:false,color_online:true});
 assert.equal(pending.label,'关闭深度');assert.equal(pending.pressed,false);assert.equal(pending.disabled,false);
 assert.equal(depthButtonView({requested:true,state:'online',active:true,color_online:true}).pressed,true);
 const closing=depthButtonView({requested:false,state:'stopping',active:true,color_online:true});
 assert.equal(closing.disabled,true);assert.equal(closing.pressed,true);
 const lost=depthButtonView({requested:true,state:'unavailable',active:false,color_online:false});
 assert.equal(lost.disabled,false);assert.equal(lost.pressed,false);
});
