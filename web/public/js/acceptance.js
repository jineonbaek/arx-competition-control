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
