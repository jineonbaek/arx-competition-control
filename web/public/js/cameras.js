import {frameFresh,cameraImagePath,chooseCameraFrame} from './camera-freshness.js';

const panel=document.getElementById('camera-panel');
const pause=document.getElementById('camera-pause');
let paused=false;
const wait=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const cards=[...panel.querySelectorAll('[data-camera]')].map(card=>({
 card,key:card.dataset.camera,img:card.querySelector('img'),
 status:card.querySelector('.camera-status'),detail:card.querySelector('.camera-detail'),
 message:card.querySelector('.camera-message'),displayed:null,anchor:null,sequence:null
}));
function unavailable(c,text){
 c.card.classList.remove('camera-online');c.status.textContent=text;
 c.message.textContent=text;c.img.hidden=true;c.message.hidden=false;
 const detail=c.card.querySelector('.camera-vision-detail');if(detail){detail.textContent='定位结果已隐藏';detail.classList.remove('valid');}
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
 const features=meta.whole_battery;const location=meta.battery?.location;
 detail.classList.toggle('valid',Boolean(features) || location?.status==='surface_valid');
 const lines=[];
 if(features){
  lines.push(`电池外框：${features.count} · 可见把手孔：${features.hole_count ?? 0}`+(features.partial_count?` · ${features.partial_count} 个目标被截断`:''));
  lines.push((features.hole_count ?? 0)>0?'蓝色 H：图像中实际检出的孔洞':'孔洞未检出或不可见，不推断孔洞位置');
 }else lines.push('孔洞与外框识别未更新 · 当前显示相机预览');
 if(c.key==='d435' && location?.status==='surface_valid' && location.bar_surface_xyz_m?.length===3){
  const [x,y,z]=location.bar_surface_xyz_m.map(v=>(v*1000).toFixed(0));
  lines.push(`G 实体表面：X ${x} / Y ${y} / Z ${z} mm（D435 相机坐标）`);
 }else if(c.key==='d435')lines.push('实体表面深度无效，未输出三维坐标');
 else lines.push('二维识别 · 未输出三维坐标');
 detail.textContent=lines.join('\n');

}
async function loop(c){
 for(;;){
  if(paused || document.hidden){await wait(250);continue;}
  try{
   let selected=await json(`/camera-live/${c.key}.json?t=${Date.now()}`);
   {
    try{
     const boxed=await json(`/camera-live/${c.key}-battery.json?t=${Date.now()}`);
     if(chooseCameraFrame(c.key,selected.meta,boxed.meta,boxed.server)===boxed.meta)selected=boxed;
    }catch(error){/* Raw preview remains available while inference is unavailable. */}
   }
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
