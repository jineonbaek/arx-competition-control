'use strict';
const $=id=>document.getElementById(id);
const labels={idle:'待命',entry:'J6 准备',lift:'升降至约 10 rad',hold:'保持'};
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
async function cameras(){for(const c of ['d435','d405','astra']){try{const m=await(await fetch('/camera-live/'+c+'.json')).json();const f=m.image_file||m.raw_image_file;if(f&&m.state==='online')$(c).src='/camera-live/'+encodeURIComponent(f);}catch(e){}}setTimeout(cameras,500);}update();cameras();
