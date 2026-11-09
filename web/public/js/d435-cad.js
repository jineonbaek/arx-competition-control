const button=document.getElementById('read'),status=document.getElementById('status'),frame=document.getElementById('frame'),rows=document.getElementById('rows');
button.onclick=async()=>{
  button.disabled=true;rows.replaceChildren();frame.hidden=true;status.textContent='正在读取…';
  try{
    const response=await fetch('/api/d435-cad',{cache:'no-store',signal:AbortSignal.timeout(4000)}),data=await response.json();
    if(!response.ok)throw Error(data.message);
    frame.src=data.image;frame.hidden=false;
    status.textContent='检出 '+data.detected+' 个标记，'+data.tags.length+' 个通过画面大小与重投影检查。';
    for(const tag of data.tags){
      const row=document.createElement('p');
      row.textContent='ID '+tag.id+' · base_link [mm]  '+tag.base_mm.join(' / ')+' · body4 [mm]  '+(tag.body4_mm?.join(' / ')||'升降反馈未配对')+' · RMS '+tag.rms_px+' px';
      rows.append(row);
    }
  }catch(err){status.textContent=err.message;}finally{button.disabled=false;}
};
button.click();
