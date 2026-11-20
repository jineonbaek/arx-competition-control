'use strict';
const button=document.getElementById('capture'),axis=document.getElementById('axis');
async function collect(path){
  button.disabled=true;axis.disabled=true;document.getElementById('state').textContent='正在采集，只读，不会运动';
  try{
    const response=await fetch(path,{method:'POST'});
    const value=await response.json();
    document.getElementById('result').textContent=JSON.stringify(value,null,2);
    document.getElementById('state').textContent=response.ok?'采集完成；结果不可直接用于运动':'采集失败';
  }catch(e){document.getElementById('state').textContent='采集连接失败';}
  finally{button.disabled=false;axis.disabled=false;}
}
button.onclick=()=>collect('/api/capture');axis.onclick=()=>collect('/api/axis');
setInterval(()=>{for(const name of ['d435','d405'])document.getElementById(name).src='/camera/'+name+'.jpg?t='+Date.now();},500);
