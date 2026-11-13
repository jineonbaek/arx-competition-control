'use strict';
const logicalNames=['左杆 · 横向','左杆 · 纵向','右杆 · 横向','右杆 · 纵向','十字键 · 横向','十字键 · 纵向'];
const controls=['Y','B','A','X','LB','RB','LT','RT','L3','R3','M1','M2'];
const axesPanel=document.querySelector('#axes');
const status=document.querySelector('#status');
let previousMode=null,axisNames=[],axisCount=0,currentMapping={buttons:{},triggers:{},logical:[]};
function makeLayout(state){
 const mapping=state.mapping||{buttons:{},triggers:{},logical:[]};
 currentMapping=mapping;axisCount=state.axes.length;
 axisNames=Array.from({length:axisCount},(_,i)=>`轴 ${i}`);
 mapping.logical.forEach((raw,i)=>axisNames[raw]=logicalNames[i]);
 Object.entries(mapping.triggers).forEach(([name,index])=>axisNames[index]=`${name} · 模拟扳机`);
 axesPanel.replaceChildren();
 for(let i=0;i<axisCount;i++)axesPanel.insertAdjacentHTML('beforeend',`<div class="axis"><div class="axis-label"><span>${axisNames[i]}</span><code>AXIS ${i}</code></div><div class="axis-values"><b id="raw-${i}">—</b><small id="norm-${i}">—</small></div><div class="track"><i id="marker-${i}" style="left:50%"></i></div></div>`);
 const chips=document.querySelector('#button-map');chips.replaceChildren();
 for(const name of controls){const trigger=mapping.triggers[name];const value=trigger===undefined?mapping.buttons[name]:`轴 ${trigger}`;chips.insertAdjacentHTML('beforeend',`<span class="chip" data-control="${name}"><b>${name}</b>${value??'?'}</span>`);}
 document.querySelectorAll('svg [data-control]').forEach(el=>{
  const name=el.dataset.control,trigger=mapping.triggers[name],index=mapping.buttons[name];
  const sub=el.querySelector('tspan');if(sub)sub.textContent=trigger===undefined?index:`轴${trigger}`;
  const label=el.querySelector('.index');
  if(label){label.textContent=(name==='M1'||name==='M2')?`同 ${state.mode==='XINPUT'?(name==='M1'?'B':'LB'):(name==='M1'?'R3':'L3')} · ${index}`:index;}
 });
 document.querySelector('#left-stick .stick-label tspan').textContent=`${mapping.logical[0]} / ${mapping.logical[1]}`;
 document.querySelector('#right-stick .stick-label tspan').textContent=`${mapping.logical[2]} / ${mapping.logical[3]}`;
 document.querySelector('.dpad .stick-label tspan').textContent=`${mapping.logical[4]} / ${mapping.logical[5]}`;
 document.querySelector('#profile-hint').textContent=state.mode==='NS'?'保留 NS 原实测中心及映射。左杆右端暂按 0 估算。':'XINPUT 使用本次实测中心与端点。LT 为模拟轴，扣过一半使能，松开停车。';
 document.querySelector('#alias-note').textContent=state.mode==='XINPUT'?'↳ M1 = B · M2 = LB（本次实测）':'↳ M1 = R3 · M2 = L3（原 NS 映射）';
}
function render(state){
 const online=state.connected;
 if(online&&(previousMode!==state.mode||axisCount!==state.axes.length)){makeLayout(state);previousMode=state.mode;}
 const c=state.control;
 document.querySelector('#control-status').textContent=c?((c.execute?'实车模式 · ':'预览模式 · ')+c.reason):'服务连接中断 · 输出状态未知';
 const command=c&&online?c.sdk_target:[0,0,0];
 ['x','y','z'].forEach((axis,i)=>document.querySelector('#cmd-'+axis).textContent=Number(command[i]).toFixed(3));
 status.className=`status ${online?'online':'offline'}`;
 status.querySelector('b').textContent=online?'手柄已连接':(state.error||'手柄未连接');
 document.querySelector('#device-mode').textContent=online?`${state.mode} 模式`:'模式未识别';
 document.body.classList.toggle('connected',online);document.body.classList.toggle('offline-view',!online);
 const m=currentMapping;
 document.querySelectorAll('[data-control]').forEach(el=>{
  const name=el.dataset.control,trigger=m.triggers[name];
  const pressed=trigger===undefined?!!state.buttons[m.buttons[name]]:state.normalized[trigger]>=.5;
  el.classList.toggle('active',online&&pressed);
 });
 const logical=online?state.logical_axes:[0,0,0,0,0,0];
 document.querySelector('#left-cap').setAttribute('transform',`translate(${logical[0]*23} ${logical[1]*23})`);
 document.querySelector('#right-cap').setAttribute('transform',`translate(${logical[2]*23} ${logical[3]*23})`);
 const directions={up:logical[5]<-.5,right:logical[4]>.5,down:logical[5]>.5,left:logical[4]<-.5};
 document.querySelectorAll('[data-dpad]').forEach(el=>el.classList.toggle('active',online&&directions[el.dataset.dpad]));
 for(let i=0;i<axisCount;i++){
  const value=online?state.normalized[i]:0;
  document.querySelector(`#raw-${i}`).textContent=online?state.axes[i]:'—';
  document.querySelector(`#norm-${i}`).textContent=online?`${value>=0?'+':''}${value.toFixed(3)}`:'—';
  const trigger=Object.values(m.triggers).includes(i);
  document.querySelector(`#marker-${i}`).style.left=`${trigger?value*100:(value+1)*50}%`;
 }
 const e=online?state.last_event:null;
 const names=e&&e.kind==='button'?Object.entries(m.buttons).filter(([,index])=>index===e.index).map(([name])=>name).join(' / '):'';
 document.querySelector('#event-title').textContent=e?(e.kind==='button'?names||`按钮 ${e.index}`:axisNames[e.index]):(online?'等待操作':'输入已清空');
 document.querySelector('#event-detail').textContent=e?(e.kind==='button'?`BUTTON ${e.index} · ${e.value?'按下':'松开'}`:`AXIS ${e.index} · ${e.value}`):(online?'拨动摇杆或按下任意按键':'连接恢复后自动继续显示');
 document.querySelector('#refresh').textContent=online?'LIVE · 20 Hz':'等待连接';
}
async function poll(){
 try{const response=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(1800)});if(!response.ok)throw Error('HTTP '+response.status);render(await response.json());}
 catch(error){render({connected:false,error:'服务连接中断',buttons:[],axes:[],normalized:[],logical_axes:[0,0,0,0,0,0]});}
 setTimeout(poll,50);
}
poll();
