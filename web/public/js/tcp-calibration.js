const byId=id=>document.getElementById(id);
const sides={left:'左臂',right:'右臂'};
let selected='left',state=null,busy=false,stopped=false;

async function request(data){
  const options=data?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)}:{cache:'no-store'};
  options.signal=AbortSignal.timeout(data?8000:2500);
  const response=await fetch('/api/tcp-calibration',options);
  const payload=await response.json();
  if(!response.ok)throw Error(payload.message||'TCP 标定服务不可用');
  return payload;
}
const fixed=(value,digits=1)=>Number.isFinite(value)?value.toFixed(digits):'—';
const triplet=(values,start=0)=>Array.isArray(values)?values.slice(start,start+3).map(value=>fixed(value)).join(' / '):'—';

function renderResult(record){
  const target=byId('tcp-result'),solution=record?.solution;
  if(!solution){target.innerHTML='<p>至少记录 6 个姿态后可以求解。</p>';return;}
  const offset=solution.tcp_offset_mm;
  target.innerHTML='';
  const grid=document.createElement('div');grid.className='tcp-result-grid';
  ['X','Y','Z'].forEach((axis,index)=>{const cell=document.createElement('div'),label=document.createElement('span'),value=document.createElement('strong');label.textContent='腕部 → TCP '+axis;value.textContent=fixed(offset[index],2)+' mm';cell.append(label,value);grid.append(cell);});
  const quality=document.createElement('p');quality.className='tcp-quality';quality.textContent='RMS '+fixed(solution.rms_mm,3)+' mm · 最大残差 '+fixed(solution.max_error_mm,3)+' mm · 秩 '+solution.rank+'/6 · 条件数 '+fixed(solution.condition,2);
  target.append(grid,quality);
}

function renderSamples(samples){
  const body=byId('tcp-samples');body.innerHTML='';
  if(!samples?.length){const row=document.createElement('tr'),cell=document.createElement('td');cell.colSpan=6;cell.textContent='尚无样本';row.append(cell);body.append(row);return;}
  samples.forEach(sample=>{
    const row=document.createElement('tr');
    const values=[sample.index,new Date(sample.captured_at*1000).toLocaleTimeString('zh-CN',{hour12:false}),triplet(sample.wrist_pose),triplet(sample.wrist_pose,3),fixed(sample.lift_raw,3)];
    values.forEach(value=>{const cell=document.createElement('td');cell.textContent=value;row.append(cell);});
    const action=document.createElement('td'),button=document.createElement('button');button.type='button';button.className='sample-remove';button.dataset.index=sample.index;button.textContent='删除';button.disabled=busy;action.append(button);row.append(action);body.append(row);
  });
}

function render(){
  const current=state?.current?.[selected],record=state?.arms?.[selected];
  byId('tcp-arm-left').classList.toggle('selected',selected==='left');
  byId('tcp-arm-right').classList.toggle('selected',selected==='right');
  byId('tcp-arm-left').setAttribute('aria-selected',String(selected==='left'));
  byId('tcp-arm-right').setAttribute('aria-selected',String(selected==='right'));
  byId('tcp-live-status').textContent=current?.available?sides[selected]+'反馈实时':current?.reason||'等待反馈';
  byId('tcp-live-pose').textContent=current?.available?'XYZ '+triplet(current.pose)+' mm · RPY '+triplet(current.pose,3)+'°':'—';
  const count=record?.sample_count||0;
  byId('tcp-count').textContent=count+' 个样本';
  byId('tcp-capture').disabled=busy||!current?.available||!byId('tcp-contact-confirm').checked;
  byId('tcp-solve').disabled=busy||count<(state?.minimum_samples||6);
  byId('tcp-save').disabled=busy||!record?.solution;
  byId('tcp-reset').disabled=busy||count===0;
  byId('tcp-saved').textContent=record?.candidate_path?'已保存：'+record.candidate_path:'';
  renderSamples(record?.samples||[]);renderResult(record);
}

async function act(op,extra={}){
  if(busy)return;
  busy=true;byId('tcp-error').textContent='';render();
  try{state=await request({op,arm:selected,...extra});if(op==='capture')byId('tcp-contact-confirm').checked=false;}
  catch(error){byId('tcp-error').textContent=error.message;}
  finally{busy=false;render();}
}

for(const side of ['left','right'])byId('tcp-arm-'+side).addEventListener('click',()=>{selected=side;byId('tcp-contact-confirm').checked=false;byId('tcp-error').textContent='';render();});
byId('tcp-contact-confirm').addEventListener('change',render);
byId('tcp-capture').addEventListener('click',()=>act('capture',{confirmed:byId('tcp-contact-confirm').checked}));
byId('tcp-solve').addEventListener('click',()=>act('solve'));
byId('tcp-save').addEventListener('click',()=>act('save',{label:byId('tcp-label').value}));
byId('tcp-reset').addEventListener('click',()=>{if(confirm('清空'+sides[selected]+'本次记录？'))act('reset');});
byId('tcp-samples').addEventListener('click',event=>{const button=event.target.closest('.sample-remove');if(button)act('remove',{index:Number(button.dataset.index)});});

async function poll(){
  while(!stopped){
    if(!busy&&!document.hidden){try{state=await request();byId('tcp-error').textContent='';}catch(error){state=null;byId('tcp-error').textContent=error.message;}render();}
    await new Promise(resolve=>setTimeout(resolve,600));
  }
}
window.addEventListener('pagehide',()=>{stopped=true;});
window.addEventListener('pageshow',event=>{if(event.persisted)window.location.reload();});
render();poll();
