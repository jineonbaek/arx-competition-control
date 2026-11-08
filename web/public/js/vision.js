const el=id=>document.getElementById(id);
const initial=new URLSearchParams(location.search).get('camera');
if(['d435','d405'].includes(initial))el('camera').value=initial;
let current=null,busy=false,lastHeartbeat=0,streamRun=null,reviewKey=null;
async function api(data){
 const response=await fetch('/api/vision',data?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data),signal:AbortSignal.timeout(10000)}:{cache:'no-store',signal:AbortSignal.timeout(3000)});
 const result=await response.json();
 if(!response.ok)throw Error(result.message||'视觉识别请求失败');
 return result;
}
function render(state){
 current=state;
 el('start').disabled=busy||state.running;
 el('stop').disabled=busy||!state.running;
 el('camera').disabled=busy||state.running;
 if(state.running)el('camera').value=state.camera;
 el('status').textContent=state.message;
 el('status').classList.toggle('error',state.status==='error');
 el('metrics').textContent=state.running&&state.sequence?
   state.fps+' FPS · 候选 '+state.count+' · '+new Date(state.captured_at*1000).toLocaleTimeString('zh-CN',{hour12:false}):'等待画面';
 if(state.running&&streamRun!==state.run_id){
   streamRun=state.run_id;
   el('stream').src='/api/vision/stream?run='+encodeURIComponent(streamRun);
 }
 const fresh=state.running&&state.sequence&&Math.abs(Date.now()/1000-state.captured_at)<3;
 el('stream').hidden=!fresh;
 el('stream-empty').hidden=Boolean(fresh);
 el('stream-empty').textContent=state.running?'模型加载中或等待新画面':'点击开始识别';
 if(!state.running&&streamRun){
   el('stream').removeAttribute('src');streamRun=null;
 }
 const q=state.qwen;
 if(q){
   el('review-time').textContent=q.captured_at?'采样 '+new Date(q.captured_at*1000).toLocaleTimeString('zh-CN',{hour12:false}):'等待采样';
   el('review-status').textContent=q.phase==='thinking'?'正在复核抽样画面…':
     q.phase==='error'?q.message:q.status==='perception_accepted'?'该采样画面复核通过':'该采样画面尚未通过';
   el('review-reason').textContent=q.review?.reason||q.selection?.reason||'';
   if(q.image_file){
     const key=state.run_id+'-'+q.source_sequence;
     if(key!==reviewKey){
       reviewKey=key;
       el('review').src='/api/vision/review?run='+encodeURIComponent(state.run_id)+'&v='+q.source_sequence;
       el('review').hidden=false;el('review-empty').hidden=true;
     }
   }
 }
}
async function act(op){
 busy=true;render(current||{status:'idle',running:false,message:'正在提交…'});
 try{
  const state=await api(op==='start'?{op,camera:el('camera').value}:{op,run_id:current.run_id});
  if(op==='start'){
    reviewKey=null;el('review').hidden=true;el('review-empty').hidden=false;
    el('review-status').textContent='等待抽样复核';el('review-reason').textContent='';
  }
  busy=false;render(state);
 }catch(error){busy=false;render(current);el('status').textContent=error.message;el('status').classList.add('error');}
}
el('start').onclick=()=>act('start');
el('stop').onclick=()=>act('stop');
async function poll(){
 try{
  const state=await api();
  render(state);
  if(state.running&&Date.now()-lastHeartbeat>4000){
   lastHeartbeat=Date.now();
   await api({op:'heartbeat',run_id:state.run_id});
  }
 }catch(error){el('status').textContent=error.message;el('status').classList.add('error');}
 setTimeout(poll,400);
}
poll();
