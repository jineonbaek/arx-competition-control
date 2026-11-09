const examples=await (await fetch('/api/examples')).json();
const select=document.querySelector('#tool-examples'),input=document.querySelector('#arxgent-input');
for(const [label,args] of Object.entries(examples)){
  select.add(new Option(label,'/tool '+label.split(' · ')[0]+' '+JSON.stringify(args)));
}
select.onchange=()=>{input.value=select.value;input.dispatchEvent(new Event('input'));input.focus();};
const job=document.querySelector('#evidence-id'),output=document.querySelector('#evidence-output');
document.querySelector('#evidence-show').onclick=()=>{
  output.replaceChildren();
  if(!/^[a-f0-9]{32}$/.test(job.value.trim())){output.textContent='请输入当前对话的任务 ID';return;}
  for(const name of ['overlay','mask','candidates']){
    const img=document.createElement('img');
    img.width=320;img.alt=name;img.src='/evidence/'+job.value.trim()+'/'+name+'.png';
    img.onerror=()=>img.replaceWith(document.createTextNode(name+' 尚无图像 '));output.append(img);
  }
};

const scene=document.querySelector('#scene-image');
document.querySelector('#scene-show').onclick=()=>{
  const id=document.querySelector('#scene-id').value.trim(),camera=document.querySelector('#scene-camera').value;
  scene.hidden=true;
  if(!/^obs_[a-f0-9]{32}$/.test(id)){output.textContent='请输入当前对话的观测 ID';return;}
  scene.dataset.id=id;scene.dataset.camera=camera;
  scene.onload=()=>{scene.hidden=false;};
  scene.onerror=()=>{scene.hidden=true;output.textContent='找不到当前对话中的画面';};
  scene.src='/scene/'+id+'/'+camera+'.jpg';
};
scene.onclick=event=>{
  if(scene.dataset.camera!=='d405')return;
  const rect=scene.getBoundingClientRect();
  const col=Math.floor((event.clientX-rect.left)*scene.naturalWidth/rect.width);
  const row=Math.floor((event.clientY-rect.top)*scene.naturalHeight/rect.height);
  input.value='/tool locate '+JSON.stringify({method:'pixels',observation_id:scene.dataset.id,points:[[row,col]]});
  input.dispatchEvent(new Event('input'));input.focus();
};
