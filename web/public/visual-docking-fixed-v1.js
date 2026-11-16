'use strict';
const $=id=>document.getElementById(id);
const labels={idle:'待命',entry:'右臂固定姿态',lift:'升降至约 10 rad',hold:'保持'};
let busy=false;
async function action(op){
  if(busy)return; busy=true; $('start').disabled=true;
  try { const r=await fetch('/api/fast-level',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({op})}); const j=await r.json(); if(!r.ok) $('reason').textContent=j.message||'请求未确认'; }
  catch(e){$('reason').textContent='控制请求未送达，请检查连接';}
  finally{busy=false;}
}
$('start').onclick=()=>action('start');
$('cancel').onclick=()=>action('cancel');
async function update(){
  try{
    const [a,b]=await Promise.all([fetch('/api/fast-level'),fetch('/api/state')]);
    if(!a.ok||!b.ok)throw Error();
    const j=await a.json(),s=await b.json();
    $('phase').textContent=(labels[j.phase]||j.phase)+' · '+j.status;
    $('reason').textContent=j.reason||'';
    const ready=s.arms?.left?.fresh&&s.arms?.right?.fresh&&s.body?.fresh&&!s.motion_reserved&&!s.competition_reserved&&!s.chassis_active;
    $('start').disabled=busy||!ready||!['idle','blocked','held','prepared','cancelled'].includes(j.status);
    if(!ready&&!j.reason)$('reason').textContent='控制权占用或反馈未就绪';
  }catch(e){$('start').disabled=true;$('reason').textContent='控制连接中断';}
  setTimeout(update,500);
}
async function cameras(){
 for(const c of ['d435','d405','astra']){
  try{
   const m=await(await fetch('/camera-live/'+c+'.json',{cache:'no-store'})).json();
   let overlay=null;
   if(c!=='astra')overlay=await(await fetch('/camera-overlay/'+c+'.json',{cache:'no-store'})).json();
   if(overlay&&!overlay.stale&&Date.now()/1000-overlay.captured_at<2){
    $(c).src='/camera-overlay/'+c+'.jpg?t='+overlay.sequence;
    $(c).title=c+' 实时标注；拍摄时间 '+new Date(overlay.captured_at*1000).toLocaleTimeString();
   }else{
    const f=m.image_file||m.raw_image_file;
    if(f&&m.state==='online')$(c).src='/camera-live/'+encodeURIComponent(f);
    $(c).title='原始画面；标注暂不可用';
   }
  }catch(e){$(c).title='画面或标注连接中断';}
 }
 setTimeout(cameras,500);
}update();cameras();
