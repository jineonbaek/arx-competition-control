const el=id=>document.getElementById(id);
const canvas=el('nav-map'),ctx=canvas.getContext('2d');
function show(data){
 const grid=data.map,pose=data.pose,route=data.route||{};
 el('nav-pose').textContent=pose?.valid?`X ${pose.x.toFixed(3)} m · Y ${pose.y.toFixed(3)} m · 朝向 ${(pose.yaw*180/Math.PI).toFixed(1)}°`:'定位尚未有效';
 el('nav-route').textContent=route.available?(route.status||'导航端已连接'):'导航端尚未就绪';
 el('nav-error').textContent=data.error||'';
 el('nav-empty').hidden=!!grid;
 if(!grid){ctx.clearRect(0,0,canvas.width,canvas.height);el('nav-map-info').textContent='';return;}
 const w=grid.width,h=grid.height;
 canvas.width=w;canvas.height=h;
 const pixels=ctx.createImageData(w,h);
 for(let y=0;y<h;y++)for(let x=0;x<w;x++){
  const value=grid.cells[y*w+x],i=((h-1-y)*w+x)*4;
  const gray=value<0?110:value>50?25:230;
  pixels.data[i]=gray;pixels.data[i+1]=gray;pixels.data[i+2]=gray;pixels.data[i+3]=255;
 }
 ctx.putImageData(pixels,0,0);
 const origin=grid.origin,angle=origin[2]||0,c=Math.cos(angle),s=Math.sin(angle);
 const pixel=p=>{const x=p[0]-origin[0],y=p[1]-origin[1];return [(c*x+s*y)/grid.resolution,h-(-s*x+c*y)/grid.resolution];};
 if(data.path?.frame===grid.frame&&data.path.points?.length){
  ctx.strokeStyle='#26a7e8';ctx.lineWidth=2;ctx.beginPath();data.path.points.forEach((p,i)=>{const [x,y]=pixel(p);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();
 }
 if(pose?.valid&&pose.frame===grid.frame){
  const [x,y]=pixel([pose.x,pose.y]),size=Math.max(4,Math.min(w,h)*.018);
  ctx.save();ctx.translate(x,y);ctx.rotate(-(pose.yaw-angle));ctx.fillStyle='#ff991f';ctx.beginPath();ctx.moveTo(size,0);ctx.lineTo(-size*.7,-size*.6);ctx.lineTo(-size*.7,size*.6);ctx.closePath();ctx.fill();ctx.restore();
 }
 el('nav-map-info').textContent=`${grid.frame} · ${w} × ${h} · 分辨率 ${grid.resolution.toFixed(3)} m`;
}
async function update(){
 try{const r=await fetch('/api/navigation-view',{cache:'no-store',signal:AbortSignal.timeout(5000)});if(!r.ok)throw Error('读取导航状态失败');show(await r.json());el('nav-connection').textContent='状态连接正常';}
 catch(error){el('nav-connection').textContent='状态连接中断';el('nav-error').textContent=error.message;el('nav-pose').textContent='定位暂不可用';ctx.clearRect(0,0,canvas.width,canvas.height);}
 setTimeout(update,600);
}
update();
