const el=id=>document.getElementById(id);
const photo=el('photo'),ctx=photo.getContext('2d');
let state={},image=null,roi=null,start=null,busy=false,polling=false;
const xyz=p=>p.map(v=>v.toFixed(1)).join(' / ');
function draw(){
  ctx.clearRect(0,0,photo.width,photo.height);
  if(image)ctx.drawImage(image,0,0);
  if(roi){ctx.strokeStyle='#ffc652';ctx.lineWidth=2;ctx.strokeRect(roi[0],roi[1],roi[2]-roi[0],roi[3]-roi[1]);}
}
function buttons(){
  el('capture').disabled=busy||state.running;
  el('estimate').disabled=busy||state.running||!roi||roi[2]-roi[0]<12||roi[3]-roi[1]<12;
  el('cancel').disabled=!state.running;
  el('gpu').textContent=state.running?'正在计算 · GPU 使用中':'模型未加载 · 显存已释放';
}
function clearResult(){
  el('overlay').hidden=true;el('mask').hidden=true;el('empty').hidden=false;
  el('metrics').replaceChildren();el('matrix').textContent='等待估计';el('result-stamp').textContent='等待估计';
  el('grip-pose-status').textContent='估计位姿后，将夹持参考点投影到画面并转换到 base_link。';
}
async function show(data){
  const previous=state.view?.id;
  state=data;
  el('status').textContent=data.message;
  if(data.view && data.view.id!==previous){
    image=new Image();image.src=data.view.image;await image.decode();
    photo.width=data.view.width;photo.height=data.view.height;roi=null;clearResult();draw();
    el('stamp').textContent=new Date(data.view.time*1000).toLocaleString('zh-CN');
    el('depth').src=data.depth_image;el('depth').hidden=false;
    el('depth-info').textContent='对齐彩色 · 有效深度 '+(100*data.valid_depth_fraction).toFixed(0)+'% · 图像与右臂时差 '+data.view.paired_ms+' ms';
  }
  if(data.result && data.status==='complete'){
    const r=data.result;
    el('overlay').src=r.overlay;el('overlay').hidden=false;el('empty').hidden=true;
    el('mask').src=r.mask_image;el('mask').hidden=false;
    el('result-stamp').textContent='快照 '+data.view.id.slice(0,8);
    const entries=[['电池中心 XYZ',xyz(r.center_base_mm)+' mm'],['模型 RPY',xyz(r.rpy_base_deg)+'°'],
      ['定位耗时',r.registration_s.toFixed(2)+' s'],['峰值已分配显存',(r.peak_allocated_mib/1024).toFixed(2)+' GiB']];
    if(r.grasp_reference){
      const g=r.grasp_reference;
      entries.push(['G 夹持中心 XYZ',xyz(g.points_base_mm.grip)+' mm'],
        ['T 指尖平面中心',xyz(g.points_base_mm.tip)+' mm'],['候选伸入量',g.insertion_mm.toFixed(0)+' mm']);
      el('grip-pose-status').textContent='约定夹姿 J6 −90°。模型凸起朝下偏差 '+g.bump_down_error_deg.toFixed(1)+'°；路径跟随 FoundationPose 变换后的电池插入轴。相对整车 +X 偏差 '+g.approach_x_error_deg.toFixed(1)+'°仅作姿态显示，不再作为路径门控。';
    }else{
      el('grip-pose-status').textContent='当前结果不含电池夹持参考点；请重新估计电池位姿。';
    }
    el('metrics').replaceChildren(...entries.map(([label,value])=>{
      const box=document.createElement('div'),dt=document.createElement('dt'),dd=document.createElement('dd');
      dt.textContent=label;dd.textContent=value;box.append(dt,dd);return box;
    }));
    el('matrix').textContent=['T_base_link_from_object（平移单位 m）',...r.T_base_from_object.map(row=>row.map(v=>v.toFixed(5)).join('  '))].join(String.fromCharCode(10));
  }
  buttons();
  if(data.running&&!polling)poll();
}
async function request(op){
  if(busy&&op!=='stop')return;
  busy=true;buttons();
  if(op==='infer'){clearResult();el('status').textContent='正在提交估计…';}
  if(op==='capture')el('status').textContent='正在获取对齐 RGB-D…';
  try{
    const body=op==='infer'?{op,id:state.view.id,roi}:op==='stop'?{op,run_id:state.run_id}:{op};
    const response=await fetch('/api/foundationpose',op?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:AbortSignal.timeout(25000)}:{cache:'no-store'});
    const data=await response.json();if(!response.ok)throw Error(data.message||'请求失败');
    await show(data);
  }catch(error){el('status').textContent=error.message;}
  finally{busy=false;buttons();}
}
async function poll(){
  polling=true;
  try{
    while(state.running){
      await new Promise(resolve=>setTimeout(resolve,600));
      const response=await fetch('/api/foundationpose',{cache:'no-store',signal:AbortSignal.timeout(5000)});
      if(!response.ok)throw Error('位姿状态读取失败');
      await show(await response.json());
    }
  }catch(error){el('status').textContent=error.message+'；任务最长 90 秒后自动结束';}
  finally{polling=false;}
}
function pixel(e){
  const b=photo.getBoundingClientRect();
  return [Math.round(Math.max(0,Math.min(1,(e.clientX-b.left)/b.width))*photo.width),
    Math.round(Math.max(0,Math.min(1,(e.clientY-b.top)/b.height))*photo.height)];
}
photo.onpointerdown=e=>{
  if(busy||state.running||!image)return;
  roi=null;clearResult();start=pixel(e);photo.setPointerCapture(e.pointerId);
};
photo.onpointermove=e=>{
  if(!start)return;const p=pixel(e);
  roi=[Math.min(start[0],p[0]),Math.min(start[1],p[1]),Math.max(start[0],p[0]),Math.max(start[1],p[1])];draw();
};
photo.onpointerup=e=>{if(!start)return;photo.onpointermove(e);start=null;buttons();};
photo.onpointercancel=()=>{start=null;roi=null;draw();buttons();};
el('capture').onclick=()=>request('capture');
el('estimate').onclick=()=>request('infer');
el('cancel').onclick=()=>request('stop');
addEventListener('pagehide',()=>{
  if(state.running)fetch('/api/foundationpose',{method:'POST',keepalive:true,headers:{'Content-Type':'application/json'},body:JSON.stringify({op:'stop',run_id:state.run_id})}).catch(()=>{});
});
request();


async function loadGripGeometry(){
  try{
    const response=await fetch('cad/battery-grip.json',{cache:'no-store'});
    if(!response.ok)throw Error('CAD 参考数据读取失败');
    const p=await response.json();
    const entries=[['候选指尖伸入',p.insertion_mm.toFixed(0)+' mm'],
      ['原始孔中心计算',p.insertion_exact_mm.toFixed(2)+' mm'],
      ['提手通孔',p.hole_size_mm.map(v=>v.toFixed(1)).join(' × ')+' mm'],
      ['提手板厚',p.handle_thickness_mm.toFixed(1)+' mm'],
      ['两指尖纵向错位',p.tip_alignment_error_mm.toFixed(3)+' mm'],
      ['槽底避让凸起',p.bump_relief_clearance_mm.toFixed(1)+' mm'],
      ['凸起左右名义余量',p.lateral_clearance_each_mm.toFixed(1)+' mm / 侧'],
      ['CAD 凸起采样落在槽内',p.recess_coverage.inside_recess_samples+' / '+p.recess_coverage.protrusion_vertex_samples],
      ['G · 电池 CAD 坐标',xyz(p.points_object_mm.grip)+' mm']];
    el('grip-geometry').replaceChildren(...entries.map(([label,value])=>{
      const box=document.createElement('div'),dt=document.createElement('dt'),dd=document.createElement('dd');
      dt.textContent=label;dd.textContent=value;box.append(dt,dd);return box;
    }));
    el('grip-status').textContent='按孔中心计算 '+p.insertion_exact_mm.toFixed(2)+' mm，向上取整至 '+p.insertion_mm.toFixed(0)+' mm。0.5 mm 网格采样未见夹指与电池穿透；这不包含无人机、机架、线缆和制造/安装误差，也不是全路径碰撞检查。';
  }catch(error){el('grip-status').textContent=error.message;}
}
loadGripGeometry();
