import * as THREE from '../vendor/lift2-preview/three.module.js';
import {OrbitControls} from '../vendor/lift2-preview/OrbitControls.js';
const el=id=>document.getElementById(id);
const photo=el('photo'),ctx=photo.getContext('2d');
let view=null,img=null,roi=null,selection=0,drag=null,busy=false;
const paths=[[0,1],[1,2],[2,3],[4,5]];
const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(48,1,.005,4);
camera.up.set(0,0,1);
let renderer,orbit,points,lines;
try{
  renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  renderer.setClearColor(0x080d16);el('viewport').append(renderer.domElement);
  orbit=new OrbitControls(camera,renderer.domElement);orbit.minDistance=.03;orbit.maxDistance=3;
  orbit.addEventListener('change',render);
  new ResizeObserver(()=>{const v=el('viewport');renderer.setSize(v.clientWidth,v.clientHeight);
    camera.aspect=v.clientWidth/Math.max(1,v.clientHeight);camera.updateProjectionMatrix();render();}).observe(el('viewport'));
}catch{el('viewport').textContent='浏览器未提供 WebGL，彩色图仍可查看候选。';}
function render(){renderer?.render(scene,camera);}
function discard(object){if(object){scene.remove(object);object.geometry.dispose();object.material.dispose();}}
function candidates(){return view?.result?.candidates||[];}
function resetCamera(){
  if(!orbit || !view?.base_from_color)return;
  const p=candidates()[selection]?.camera_m||[0,0,.32];
  const transform=new THREE.Matrix4().set(...view.base_from_color.flat());
  orbit.target.set(...p).applyMatrix4(transform);
  camera.position.set(p[0]-.055,p[1]-.07,p[2]-.36).applyMatrix4(transform);
  orbit.update();render();
}
el('reset').onclick=resetCamera;
function cloud(){
  discard(points);points=null;
  if(!renderer||!view?.points?.length)return;
  const coords=[],colors=[];
  for(const [x,y,z,r,g,b] of view.points){coords.push(x,y,z);const c=new THREE.Color().setRGB(r/255,g/255,b/255,THREE.SRGBColorSpace);colors.push(c.r,c.g,c.b);}
  const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.Float32BufferAttribute(coords,3));geo.setAttribute('color',new THREE.Float32BufferAttribute(colors,3));
  points=new THREE.Points(geo,new THREE.PointsMaterial({size:.0025,vertexColors:true}));scene.add(points);
}
function draw(){
  ctx.fillStyle='#080d16';ctx.fillRect(0,0,photo.width,photo.height);if(img)ctx.drawImage(img,0,0,photo.width,photo.height);
  const box=roi;if(box){ctx.strokeStyle='#ffc652';ctx.lineWidth=2;ctx.setLineDash([6,4]);ctx.strokeRect(box[0],box[1],box[2]-box[0],box[3]-box[1]);ctx.setLineDash([]);}
  const g=candidates()[selection];if(!g)return;
  for(const [a,b] of paths){if(!g.pixels[a]||!g.pixels[b])continue;
    ctx.beginPath();ctx.moveTo(...g.pixels[a]);ctx.lineTo(...g.pixels[b]);ctx.strokeStyle='#10252e';ctx.lineWidth=6;ctx.stroke();ctx.strokeStyle='#37eddc';ctx.lineWidth=3;ctx.stroke();}
}
function details(){
  const all=candidates(),g=all[selection];discard(lines);lines=null;
  if(g&&renderer){const geo=new THREE.BufferGeometry().setFromPoints(paths.flatMap(([a,b])=>[a,b].map(i=>new THREE.Vector3(...g.lines_base[i]))));
    lines=new THREE.LineSegments(geo,new THREE.LineBasicMaterial({color:0x41ffdf,depthTest:false}));lines.renderOrder=1;scene.add(lines);}
  el('metrics').replaceChildren();
  if(g){const entries=[['模型评分',g.score.toFixed(3)],['建议开口',g.width_mm.toFixed(1)+' mm'],['抓取深度',g.depth_mm.toFixed(1)+' mm'],['中心 · base_link (X / Y / Z)',g.base_mm.map(n=>n.toFixed(1)).join(' / ')+' mm'],['接近方向 · base_link',g.approach_base.map(n=>n.toFixed(2)).join(' / ')]];
    for(const [label,value] of entries){const box=document.createElement('div'),dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=label;dd.textContent=value;box.append(dt,dd);el('metrics').append(box);}}
  el('candidate').value=String(selection);draw();render();buttons();
}
function buttons(){const n=candidates().length;
  el('capture').disabled=busy;el('infer').disabled=busy||!validRoi();el('candidate').disabled=busy||!n;
  el('prev').disabled=busy||selection<=0;el('next').disabled=busy||selection>=n-1;}
function select(i){selection=i;details();}
el('candidate').onchange=e=>select(Number(e.target.value));el('prev').onclick=()=>select(selection-1);el('next').onclick=()=>select(selection+1);
function validRoi(){return roi&&roi[2]-roi[0]>=12&&roi[3]-roi[1]>=12;}
function xy(e){const r=photo.getBoundingClientRect();return [Math.max(0,Math.min(photo.width,Math.round((e.clientX-r.left)/r.width*photo.width))),Math.max(0,Math.min(photo.height,Math.round((e.clientY-r.top)/r.height*photo.height)))];}
photo.onpointerdown=e=>{if(!img||busy)return;drag=xy(e);photo.setPointerCapture(e.pointerId);
  if(view.result){view={...view,result:null};el('counts').textContent='选框已更改';el('candidate').replaceChildren();selection=0;}
  roi=[...drag,...drag];details();};
photo.onpointermove=e=>{if(!drag)return;const p=xy(e);roi=[Math.min(drag[0],p[0]),Math.min(drag[1],p[1]),Math.max(drag[0],p[0]),Math.max(drag[1],p[1])];draw();};
photo.onpointerup=photo.onpointercancel=()=>{if(!drag)return;drag=null;buttons();status(validRoi()?'已选定目标，可以生成候选。':'请拖出一个更大的选框。');};
function status(message,error=false){el('status').textContent=message;el('status').classList.toggle('error',error);}
async function show(v){
  if(!v.id){status('点击「采集新画面」开始。');buttons();return;}
  const loaded=new Image();loaded.src=v.image;await loaded.decode();
  view=v;img=loaded;roi=v.roi;selection=0;photo.width=v.width;photo.height=v.height;
  el('stamp').textContent=new Date(v.time*1000).toLocaleTimeString()+' · '+v.width+'×'+v.height;
  el('candidate').replaceChildren(...candidates().map((g,i)=>new Option('候选 '+(i+1)+' · '+g.score.toFixed(3),String(i))));
  el('counts').textContent=v.result?'去重后 '+v.result.after_nms+' 个 · 显示 '+candidates().length+' 个':'尚未推理';
  cloud();details();resetCamera();status(v.result?(candidates().length?'推理完成 · '+v.result.elapsed_s+' 秒':'当前选区没有通过点云过滤的候选，调整选框或重新采集。'):'已冻结 RGB-D，拖动框选电池。');
}
async function run(op){
  busy=true;buttons();status(op==='capture'?'正在采集 D405 对齐深度…':op==='infer'?'正在运行 GraspNet…':'正在读取上次预览…');
  try{const args=op?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(op==='capture'?{op}:{op,id:view.id,roi})}:{cache:'no-store'};
    const reply=await fetch('/api/grasp-preview',{...args,signal:AbortSignal.timeout(op==='infer'?100000:25000)});
    const v=await reply.json();if(!reply.ok)throw Error(v.message||'请求失败');await show(v);
  }catch(err){status(err.message,true);}finally{busy=false;buttons();}
}
el('capture').onclick=()=>run('capture');el('infer').onclick=()=>run('infer');run();
