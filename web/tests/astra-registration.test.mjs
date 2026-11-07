import test from 'node:test';
import assert from 'node:assert/strict';
import {depthDetails} from '../public/js/camera-freshness.js';

test('depth availability and calibration readiness are separate',()=>{
 const raw={state:'online',width:640,height:480,valid_fraction:.8,aligned_to_color:false,calibration:{status:'required'}};
 assert.match(depthDetails(raw),/未对齐彩色/);
 assert.match(depthDetails(raw),/标定参数不可用/);
 assert.match(depthDetails({...raw,calibration:{status:'error',message:'wrong serial'}}),/wrong serial/);
 assert.match(depthDetails({...raw,aligned_to_color:true,calibration:{status:'loaded',validation:'pending_physical_check'}}),/待实拍验证/);
 assert.doesNotMatch(depthDetails({...raw,calibration:undefined}),/undefined/);
 assert.doesNotMatch(depthDetails({state:'disabled'}),/NaN/);
});
