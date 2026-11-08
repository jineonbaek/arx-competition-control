const el=id=>document.getElementById(id);
let state=null,busy=false,generation=0,approvedSession=null,clock=NaN;
const names=['J1','J2','J3','J4','J5','J6'];
const degrees=q=>q.map((v,i)=>names[i]+' '+(v*180/Math.PI).toFixed(1)+'°').join(' / ');
async function request(data) {
  const response=await fetch('/api/handeye',data?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data),signal:AbortSignal.timeout(10000)}:{cache:'no-store',signal:AbortSignal.timeout(3000)});
  const json=await response.json();
  if(!response.ok)throw Error(json.message||'标定服务不可用');
  clock=performance.now()-json.time*1000;
  return json;
}
function display() {
  const p=state?.pending;
  const valid=p && p.ticket && performance.now()-clock < p.expires*1000;
  if(approvedSession!==state?.session){el('step-confirm').checked=false;approvedSession=state?.session;}
  el('handeye-start').disabled=busy || state?.active || !state?.board_configured || !valid || !el('step-confirm').checked || el('robot-preview').dataset.loaded!=='true';
  el('step-next').disabled=busy || state?.active || !state?.session || state.cursor>=state.total;
  el('step-begin').disabled=el('step-probe').disabled=busy || state?.active;
  el('step-stop').disabled=!state?.session || (!state.active && !p);
  el('step-result').textContent=state?.active?'执行中，等待停稳和标记检测':p?'候选 '+p.point+' / '+state.total+' · '+(valid?(state.board_configured?'待人工点击':'仅预览，请设置标记参数'):'确认已失效'):
    state?.session?'当前没有待执行的点':'等待生成候选';
  el('step-detail').textContent=p?'整段路径间隙下界 '+p.check.lower_mm+' mm · 要求 > '+p.check.margin_mm+' mm · 最近部件 '+p.check.link+'\n目标 '+degrees(p.target):'';
  const history=(state?.history||[]).slice().reverse().map(h=>(h.point?'点 '+h.point+' · ':'')+h.status+' · '+h.message);
  el('step-history').textContent=history.join('\n') || '尚无采样记录';
  window.dispatchEvent(new CustomEvent('handeye-preview',{detail:valid?p:null}));
}
async function act(op){
  if(busy)return;
  busy=true;generation++;el('step-error').textContent='';display();
  try{
    const data={op,session:state?.session};
    if(op==='begin' || op==='probe'){
      if(el('board-dict').value && (!el('board-id').value || !el('board-mm').value))throw Error('请填写标记编号和边长');
      Object.assign(data,{dictionary:el('board-dict').value,marker:Number(el('board-id').value),mm:Number(el('board-mm').value)});
    }
    if(op==='execute')Object.assign(data,{ticket:state.pending?.ticket,confirmed:el('step-confirm').checked});
    state=await request(data);
  }catch(err){el('step-error').textContent=err.message;state=null;}
  finally{busy=false;display();}
}
for(const [id,op] of [['step-probe','probe'],['step-begin','begin'],['step-next','next'],['handeye-start','execute'],['step-stop','cancel']])
  el(id).addEventListener('click',()=>act(op));
el('step-confirm').addEventListener('change',display);
async function poll(){
  if(!busy && !document.hidden){
    const g=generation;
    try{const value=await request();if(g===generation&&!busy)state=value;}
    catch{if(g===generation&&!busy)state=null;}
    display();
  }
  setTimeout(poll,450);
}
poll();
