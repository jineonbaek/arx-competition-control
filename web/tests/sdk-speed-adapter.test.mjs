import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
const {ChassisOutput}=await import('data:text/javascript;base64,'+readFileSync(process.argv[2]??new URL('../public/js/chassis-output.js',import.meta.url)).toString('base64'));
const c=new ChassisOutput('web-test',()=>10);c.begin();
const out=c.move({x:.92,y:0,omega:1.8},c.generation,.4);
assert.ok(Math.abs(out.x*.4-.92)<1e-12);assert.ok(Math.abs(out.omega*.4-1.8)<1e-12);
assert.equal(out.type,'chassis');assert.equal(c.end().type,'chassis.end_output');
assert.throws(()=>c.move({x:1,y:0,omega:0}));
console.log('SDK adapter and task-scoped end: PASS');
