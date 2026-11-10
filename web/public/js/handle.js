import * as THREE from '../vendor/lift2-preview/three.module.js';
import {OrbitControls} from '../vendor/lift2-preview/OrbitControls.js';

const el=id=>document.getElementById(id);
const photo=el('photo'), ctx=photo.getContext('2d');
let view=null, image=null, roi=null, line=null, start=null, busy=false;
const scene=new THREE.Scene(), camera=new THREE.PerspectiveCamera(48,1,.005,5);
camera.up.set(0,0,1);
let renderer, orbit, cloud;
const markers=new THREE.Group();
scene.add(markers);
function render(){if(renderer)renderer.render(scene,camera);}
try {
  renderer=new THREE.WebGLRenderer({antialias:true});
  renderer.setClearColor(0x080d16);
  el('viewport').append(renderer.domElement);
  orbit=new OrbitControls(camera,renderer.domElement);
  orbit.minDistance=.025;orbit.maxDistance=3;
  orbit.addEventListener('change',render);
  new ResizeObserver(()=>{
    const box=el('viewport');
    renderer.setSize(box.clientWidth,box.clientHeight);
    camera.aspect=box.clientWidth/Math.max(1,box.clientHeight);
    camera.updateProjectionMatrix();render();
  }).observe(el('viewport'));
} catch {el('viewport').textContent='WebGL 不可用，可查看左侧彩色图。';}
function reset(){
  if(!orbit || !view?.base_from_color)return;
  const m=new THREE.Matrix4().set(...view.base_from_color.flat());
  const center=line ? new THREE.Vector3(...line.feature_mm).multiplyScalar(.001)
                    : new THREE.Vector3(0,0,.32).applyMatrix4(m);
  orbit.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(-.24,-.25,.14));orbit.update();
}
el('reset').onclick=reset;
function dispose(obj){obj.geometry.dispose();obj.material.dispose();}
function setCloud(){
  if(!renderer)return;
  if(cloud){scene.remove(cloud);dispose(cloud);}
  const coords=[],colors=[];
  for(const [x,y,z,r,g,b] of view.points||[]){
    coords.push(x,y,z);const c=new THREE.Color((r<<16)|(g<<8)|b);colors.push(c.r,c.g,c.b);
  }
  const geom=new THREE.BufferGeometry();
  geom.setAttribute('position',new THREE.Float32BufferAttribute(coords,3));
  geom.setAttribute('color',new THREE.Float32BufferAttribute(colors,3));
  cloud=new THREE.Points(geom,new THREE.PointsMaterial({size:.0025,vertexColors:true}));
  scene.add(cloud);reset();render();
}
function draw(){
  ctx.clearRect(0,0,photo.width,photo.height);
  if(image)ctx.drawImage(image,0,0);
  if(roi){ctx.strokeStyle='#ffc652';ctx.lineWidth=2;ctx.strokeRect(roi[0],roi[1],roi[2]-roi[0],roi[3]-roi[1]);}
  for(const child of [...markers.children]){markers.remove(child);dispose(child);}
  if(line){
    const colors=['#42e7ec','#ff9256'];
    const pts=[line.pre_mm,line.feature_mm].map(p=>new THREE.Vector3(...p).multiplyScalar(.001));
    markers.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),new THREE.LineBasicMaterial({color:colors[0],depthTest:false})));
    const [a,b]=line.pixels;
    ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.strokeStyle=colors[0];ctx.lineWidth=3;ctx.stroke();
    pts.forEach((p,i)=>{
      const mark=new THREE.Mesh(new THREE.SphereGeometry(.004),new THREE.MeshBasicMaterial({color:colors[i],depthTest:false}));
      mark.position.copy(p);mark.renderOrder=2;markers.add(mark);
      ctx.beginPath();ctx.arc(...line.pixels[i],5,0,2*Math.PI);ctx.fillStyle=colors[i];ctx.fill();
      ctx.strokeStyle='#182332';ctx.lineWidth=1;ctx.stroke();
    });
  }
  render();
}
function buttons(){
  el('capture').disabled=busy;el('distance').disabled=busy;
  el('plan').disabled=busy||!roi||!el('distance').validity.valid||!el('distance').value;
  el('teach').disabled=busy||!line||!el('confirm').checked;
}
function clearLine(){
  line=null;el('metrics').replaceChildren();el('profile').textContent='';
  el('quality').textContent='等待计算。';el('confirm').checked=false;buttons();draw();
}
const xyz=p=>p.map(v=>v.toFixed(1)).join(' / ');
function details(){
  const entries=[['预抓参考点',line.pre_mm],['夹持参考点',line.feature_mm]];
  if(line.teach)entries.push(['腕部预抓点',line.wrist_pre_mm],['腕部夹持终点',line.wrist_end_mm]);
  el('metrics').replaceChildren(...entries.map(([label,p])=>{
    const box=document.createElement('div'),dt=document.createElement('dt'),dd=document.createElement('dd');
    dt.textContent=label;dd.textContent=xyz(p)+' mm';box.append(dt,dd);return box;
  }));
  el('quality').textContent='前沿支持点 '+line.inliers+' / '+line.valid+' 个有效深度像素 · 沿 +X，Y、Z 固定';
  el('profile').textContent=line.teach?'已示教 · 腕部偏移 '+xyz(line.teach.offset_mm)+' mm · 适用 RPY '+xyz(line.teach.wrist_pose.slice(3))+'°':'尚未示教。当前显示提手参考线段。';
  draw();
}
async function request(op){
  if(busy)return;
  busy=true;buttons();el('status').textContent='读取中…';
  try{
    const payload={op,id:view?.id,roi,clearance_mm:el('distance').valueAsNumber,confirmed:el('confirm').checked};
    const response=await fetch('/api/handle-preview',op?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload),signal:AbortSignal.timeout(30000)}:{cache:'no-store'});
    const data=await response.json();
    if(!response.ok)throw Error(data.message||'请求失败');
    if(!op || op==='capture'){
      if(!data.id){el('status').textContent='请采集新画面。';return;}
      const img=new Image();img.src=data.image;await img.decode();view=data;image=img;roi=null;
      photo.width=view.width;photo.height=view.height;clearLine();setCloud();
      el('stamp').textContent=new Date(view.time*1000).toLocaleString('zh-CN');
      el('status').textContent='已加载冻结快照，请框选提手。';
    }else{
      line=data;details();el('confirm').checked=false;
      el('status').textContent=op==='teach'?'已记录示教偏移。':'预览完成。';
    }
  }catch(err){clearLine();el('status').textContent=err.message;}
  finally{busy=false;buttons();}
}
function pixel(e){
  const r=photo.getBoundingClientRect();
  return [Math.round(Math.max(0,Math.min(1,(e.clientX-r.left)/r.width))*photo.width),
          Math.round(Math.max(0,Math.min(1,(e.clientY-r.top)/r.height))*photo.height)];
}
function drag(e){
  const p=pixel(e);roi=[Math.min(start[0],p[0]),Math.min(start[1],p[1]),Math.max(start[0],p[0]),Math.max(start[1],p[1])];draw();
}
photo.onpointerdown=e=>{if(busy||!view)return;roi=null;clearLine();start=pixel(e);photo.setPointerCapture(e.pointerId);};
photo.onpointermove=e=>{if(start)drag(e);};
photo.onpointerup=e=>{if(!start)return;drag(e);start=null;buttons();};
photo.onpointercancel=()=>{start=null;roi=null;clearLine();};
el('distance').oninput=clearLine;el('confirm').onchange=buttons;
el('capture').onclick=()=>request('capture');el('plan').onclick=()=>request('plan');el('teach').onclick=()=>request('teach');
request();
