const el=id=>document.getElementById(id);
let state=null,busy=false,generation=0,approvedSession=null,clock=NaN;
const names=['J1','J2','J3','J4','J5','J6'];
const degrees=q=>q.map((v,i)=>names[i]+' '+(v*180/Math.PI).toFixed(1)+'°').join(' / ');
async function request(data) {
  const response=await fetch('/api/handeye',data?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data),signal:AbortSignal.timeout(10000)}:{cache:'no-store',signal:AbortSignal.timeout(3000)});
  const json=await response.json();
  if(!response.ok)throw Error(json.message||'标定服务不可用');
  if(Number.isFinite(json.time))clock=performance.now()-json.time*1000;
  return json;
}
function display() {
  const p=state?.pending;
  const valid=p && p.ticket && performance.now()-clock < p.expires*1000;
  if(approvedSession!==state?.session){el('step-confirm').checked=false;approvedSession=state?.session;}
  el('handeye-start').disabled=busy || state?.active || !state?.board_configured || !p?.profile || !Number.isFinite(p?.wrist_rise_mm) || !valid || !el('step-confirm').checked || el('robot-preview').dataset.loaded!=='true';
  el('measure-check').disabled=checking || busy || !state || state.active || Boolean(p);
  el('step-next').disabled=busy || state?.active || !state?.session || state.cursor>=state.total;
  el('step-begin').disabled=el('step-probe').disabled=busy || state?.active;
  el('step-stop').disabled=!state?.session || (!state.active && !p);
  el('step-result').textContent=state?.active?'标定程序已请求仅彩色采样，等待停稳和标记检测':p?'候选 '+p.point+' / '+state.total+' · '+(valid?(state.board_configured?'待人工点击':'仅预览，请设置标记参数'):'确认已失效'):
    state?.session?'当前没有待执行的点':'等待生成候选';
  el('step-detail').textContent=p?(Number.isFinite(p.wrist_rise_mm)?(p.profile==='supplement'?'当前腕姿补采':p.point===1?'抬腕准备点':'抬腕后采样')+' · 相对起姿腕高变化 '+p.wrist_rise_mm+' mm（模型）':'当前网页后端仍使用旧候选，请重载网页服务')+'\n到中央立柱的整段间隙下界 '+p.check.lower_mm+' mm · 要求 > '+p.check.margin_mm+' mm · 最近部件 '+p.check.link+'\n目标 '+degrees(p.target):'';
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
      Object.assign(data,{layout:el('board-layout').value,profile:el('pose-profile').value,dictionary:el('board-dict').value,marker:Number(el('board-id').value),mm:Number(el('board-mm').value)});
    }
    if(op==='execute')Object.assign(data,{ticket:state.pending?.ticket,confirmed:el('step-confirm').checked});
    state=await request(data);
  }catch(err){el('step-error').textContent=err.message;state=null;}
  finally{busy=false;display();}
}
for(const [id,op] of [['step-probe','probe'],['step-begin','begin'],['step-next','next'],['handeye-start','execute'],['step-stop','cancel']])
  el(id).addEventListener('click',()=>act(op));
function updateBoardFields(){
  const fixed=Boolean(el('board-layout').value);
  for(const [id,v] of [['board-dict','5X5_100'],['board-id','26'],['board-mm','11.25']]){
    if(fixed)el(id).value=v;
    el(id).disabled=fixed;
  }
}
el('board-layout').addEventListener('change',updateBoardFields);
updateBoardFields();
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

let checking=false;
const axes=['x','y','z'];
const xyz=values=>axes.map((axis,i)=>axis.toUpperCase()+' '+(values[i]==null?'—':values[i].toFixed(1))).join(' / ');
el('measure-check').addEventListener('click',async()=>{
  if(checking)return;
  checking=true;
  display();
  const output=el('physical-result');
  output.textContent='正在配对 D405 彩色帧与右臂反馈…';
  try{
    const reference=axes.map(axis=>{
      const input=el('check-'+axis);
      if(input.validity.badInput)throw Error('实测值需要填写数字');
      return input.value===''?null:input.valueAsNumber;
    });
    const result=await request({op:'verify_point',reference_mm:reference});
    const lines=['标记中心相对右肩（mm） '+xyz(result.shoulder_mm),
                 '三帧位置最大差 '+result.spread_mm.toFixed(1)+' mm'];
    if(result.delta_mm){
      lines.push('预测减实测（mm） '+xyz(result.delta_mm));
      if(result.error_mm!==null)lines.push('三维位置误差 '+result.error_mm.toFixed(1)+' mm');
      lines.push('实测记录 '+result.check_id);
    }else lines.push('预览 · 等待独立实测值');
    output.textContent=lines.join('\n');
  }catch(err){output.textContent=err.message;}
  finally{checking=false;display();}
});

poll();
