import {frameFresh,cameraImagePath,depthButtonView,depthDetails} from './camera-freshness.js';

const panel=document.getElementById('camera-panel');
const pause=document.getElementById('camera-pause');
let paused=false;
const wait=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const cards=[...panel.querySelectorAll('[data-camera]')].map(card=>({
 card,key:card.dataset.camera,img:card.querySelector('img'),
 status:card.querySelector('.camera-status'),detail:card.querySelector('.camera-detail'),
 message:card.querySelector('.camera-message'),depthButton:card.querySelector('[data-depth-toggle]'),depthStatus:card.querySelector('.camera-depth-status'),displayed:null,anchor:null,sequence:null
}));
function unavailable(c,text){
 c.card.classList.remove('camera-online');c.status.textContent=text;
 c.message.textContent=text;c.img.hidden=true;c.message.hidden=false;
 const detail=c.card.querySelector('.camera-vision-detail');if(detail){detail.textContent='画面已暂停显示';detail.classList.remove('valid');}
}
function serverNow(c){return c.anchor ? c.anchor.server+performance.now()-c.anchor.local : NaN;}
async function json(url){
 const controller=new AbortController();const timer=setTimeout(()=>controller.abort(),1800);
 try{
  const r=await fetch(url,{cache:'no-store',signal:controller.signal});
  if(!r.ok)throw new Error('视频服务不可用');
  const meta=await r.json();return {meta,server:Date.parse(r.headers.get('Date'))};
 }finally{clearTimeout(timer);}
}
async function show(c,meta){
 const next=document.createElement('img');next.alt=c.img.alt;
 await new Promise((resolve,reject)=>{
  const finish=(error)=>{clearTimeout(timer);next.onload=null;next.onerror=null;error?reject(error):resolve();};
  const timer=setTimeout(()=>finish(new Error('画面读取超时')),1800);
  next.onload=()=>finish();next.onerror=()=>finish(new Error('画面读取失败'));
  next.src=cameraImagePath(c.key,meta)+`?v=${encodeURIComponent(meta.captured_at)}-${meta.sequence}`;
 });
 if(paused || document.hidden || !frameFresh(meta,serverNow(c)))return;
 c.img.replaceWith(next);c.img=next;
 c.displayed=meta;c.sequence=meta.sequence;c.img.hidden=false;c.message.hidden=true;
 c.card.classList.add('camera-online');c.status.textContent='实时';
 c.detail.textContent='640 × 480 · 更新 '+new Date(meta.captured_at*1000).toLocaleTimeString('zh-CN',{hour12:false});
 let detail=c.card.querySelector('.camera-vision-detail');
 if(!detail){detail=document.createElement('p');detail.className='camera-vision-detail';c.card.append(detail);}
 const depth=meta.depth;
 detail.classList.toggle('valid',depth?.state==='online');
 detail.textContent=depthDetails(depth);

}
async function loop(c){
 for(;;){
  if(paused || document.hidden){await wait(250);continue;}
  try{
   const selected=await json(`/camera-live/${c.key}.json?t=${Date.now()}`);
   const {meta,server}=selected;
   c.anchor={server,local:performance.now()};
   if(!frameFresh(meta,serverNow(c))){
    unavailable(c,meta.state==='connecting'?'正在连接':'画面离线');
    c.detail.textContent=meta.message || '画面已停止更新，正在重试';c.sequence=null;
   }else if(c.sequence!==meta.sequence || !c.card.classList.contains('camera-online'))await show(c,meta);
  }catch(error){unavailable(c,'画面离线');c.detail.textContent='等待视频服务，自动重试';c.sequence=null;}
  await wait(180);
 }
}
pause.addEventListener('click',()=>{
 paused=!paused;pause.textContent=paused?'继续显示':'暂停显示';pause.setAttribute('aria-pressed',String(paused));
 for(const c of cards){unavailable(c,paused?'已暂停显示':'正在连接');c.sequence=null;}
});
setInterval(()=>{
 for(const c of cards){
  if(paused || document.hidden)unavailable(c,paused?'已暂停显示':'页面已隐藏');
  else if(c.displayed && !frameFresh(c.displayed,serverNow(c)))unavailable(c,'画面已过期');
 }
},300);
for(const c of cards)loop(c);

let depthStates={};const depthBusy=new Set();const depthErrors={};
function renderDepth(){
 for(const c of cards){
  const view=depthButtonView(depthStates[c.key]);
  c.depthButton.textContent=view.label;
  c.depthButton.disabled=view.disabled || depthBusy.has(c.key);
  c.depthButton.setAttribute('aria-pressed',String(view.pressed));
  c.depthStatus.textContent=depthBusy.has(c.key)?'正在提交深度开关请求':(depthErrors[c.key] || view.hint);
 }
}
for(const c of cards)c.depthButton.addEventListener('click',async()=>{
 const status=depthStates[c.key];if(!status || depthBusy.has(c.key))return;
 delete depthErrors[c.key];depthBusy.add(c.key);renderDepth();
 const controller=new AbortController();const timer=setTimeout(()=>controller.abort(),3000);
 try{
  const response=await fetch('/api/camera/depth',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({camera:c.key,enabled:!status.requested}),signal:controller.signal});
  const data=await response.json();if(!response.ok)throw new Error(data.message || '深度开关请求失败');
  depthStates=data.cameras;
 }catch(error){depthErrors[c.key]=error.message;}
 finally{clearTimeout(timer);depthBusy.delete(c.key);renderDepth();}
});
async function depthLoop(){
 for(;;){
  try{const {meta}=await json('/api/camera/depth');depthStates=meta.cameras;}
  catch(error){depthStates={};}
  renderDepth();await wait(700);
 }
}
depthLoop();
