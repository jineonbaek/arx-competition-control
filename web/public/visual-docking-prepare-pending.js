'use strict';
const $=id=>document.getElementById(id);
const labels={idle:'待命',entry:'固定姿态与翻转',lift:'升降',correct:'视觉修正',observe:'识别与规划',align:'仓外对位',insert:'浅入',hold:'保持'};
$('cancel').onclick=async()=>{
  try { const r=await fetch('/api/fast-level',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({op:'cancel'})}); const j=await r.json(); if(!r.ok) $('reason').textContent=j.message||'停止请求未确认'; }
  catch(e){$('reason').textContent='停止请求未送达，请使用现场急停';}
};
async function update(){try{const r=await fetch('/api/fast-level');if(!r.ok)throw Error();const j=await r.json();$('phase').textContent=(labels[j.phase]||j.phase)+' · '+j.status;$('reason').textContent=j.reason||'';}catch(e){$('reason').textContent='控制连接中断';}setTimeout(update,500);}
async function cameras(){for(const c of ['d435','d405','astra']){try{const m=await(await fetch('/camera-live/'+c+'.json')).json();const f=m.image_file||m.raw_image_file;if(f&&m.state==='online')$(c).src='/camera-live/'+encodeURIComponent(f);}catch(e){}}setTimeout(cameras,500);}update();cameras();
