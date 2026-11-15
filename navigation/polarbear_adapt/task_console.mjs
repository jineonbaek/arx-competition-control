const STORAGE_KEY = 'navigation.handoff.console';
const labels = {notified:'通知已生成',notification_read:'调用方读取通知',accept:'调用方确认接管',complete:'收到完成信号',fail:'作业报告失败',cancelling:'导航请求取消',cancelled:'确认停止并归还控制权',needs_attention:'交接需要处理',acknowledged:'导航确认交接'};
const phases = {notified:'等待调用方接管',running:'作业进行中',resume_pending:'已报完成，等待导航验收',cancelling:'正在取消，等待停止确认',cancelled:'已报取消，等待导航确认',acknowledged:'交接已确认',needs_attention:'需要人工处理'};
const sameVisit = (a,b) => a && b && ['mission_id','visit_id','point_id'].every(k => a[k]===b[k]);

export function deriveView(state,owned,fresh,checks={}) {
 const a=state?.task, status=a?.status;
 const events=(state?.events||[]).filter(e => e.visit_id===a?.visit_id);
 const has=event=>events.some(e=>e.event===event);
 const completed=status==='acknowledged' && a.outcome==='completed';
 const cancelled=status==='acknowledged' && a.outcome==='cancelled';
 const usable=Boolean(fresh && owned && state?.worker_ready);
 const accepted=has('accept') || ['running','resume_pending'].includes(status) || completed;
 const navigation=cancelled?'取消已确认':completed?'交接已确认':status==='acknowledged'?'交接已确认（旧记录）':['resume_pending','cancelled','cancelling'].includes(status)?'等待导航验收':a?'等待作业完成':'等待到点';
 const work=({notified:'未接管',running:'作业进行中',resume_pending:'已收到完成信号',cancelling:'正在取消',cancelled:'已收到取消回执',needs_attention:'需要人工处理'})[status] || (cancelled?'已取消':completed?'作业已确认完成':'未接管');
 return {
  completed,cancelled,notification:a?'已生成到点通知':'尚未生成',work,navigation,
  mode:!fresh?'连接过期 · 操作已禁用':usable?'本页人工联调':state?.worker_ready?'其他调用方在线 · 只读监控':'只读监控 · 可登记联调',
  canRegister:Boolean(fresh && !state?.worker_ready),canAccept:usable&&status==='notified',
  canComplete:Boolean(usable&&status==='running'&&checks.work&&checks.released&&checks.ready),
  canFail:usable&&['notified','running'].includes(status),
  canCancel:Boolean(usable&&status==='cancelling'&&checks.released&&checks.stopped),
  steps:[
   {label:'导航到点停稳，已生成通知',done:Boolean(a)},
   {label:'调用方已读取通知接口',done:Boolean(a?.first_read_at_ms)||has('notification_read')||accepted},
   {label:'调用方已确认接管',done:accepted},
   {label:'完成信号已被导航收到',done:has('complete')||status==='resume_pending'||completed},
   {label:cancelled?'取消已确认，不恢复旧目标':'导航独立验收并确认交接',done:completed||cancelled}
  ]
 };
}

export class ManualClient {
 constructor({fetch=globalThis.fetch.bind(globalThis),storage=globalThis.sessionStorage,uuid=()=>globalThis.crypto.randomUUID(),clock=Date.now}={}) {
  this.fetch=fetch;this.storage=storage;this.uuid=uuid;this.clock=clock;this.state=null;this.stateAt=-Infinity;this.authAt=-Infinity;this.authenticated=false;
  try {this.session=JSON.parse(storage.getItem(STORAGE_KEY)||'null');} catch {this.session=null;}
  if (!this.session || typeof this.session.instance_id!=='string') this.session={instance_id:uuid(),token:null,pending:null};
 }
 get fresh(){return Boolean(this.state && this.clock()-this.stateAt<1500);}
 get owned(){return Boolean(this.session.token && this.authenticated && this.clock()-this.authAt<1800);}
 save(){this.storage.setItem(STORAGE_KEY,JSON.stringify(this.session));}
 async request(path,value,auth=false) {
  const headers={},options={method:value===undefined?'GET':'POST',headers,cache:'no-store'};
  if(auth){if(!this.session.token)throw new Error('请先登记本页人工联调');headers.Authorization='Bearer '+this.session.token;}
  if(value!==undefined){headers['Content-Type']='application/json';options.body=JSON.stringify(value);}
  const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),1000);options.signal=abort.signal;
  try {
   const response=await this.fetch(path,options),data=await response.json();
   if(!response.ok){const error=new Error(data.error||`HTTP ${response.status}`);error.status=response.status;throw error;}
   return data;
  } finally {clearTimeout(timer);}
 }
 async refresh(){
  const state=await this.request('/v1/state');this.state=state;this.stateAt=this.clock();
  if(this.session.token){
   try {await this.request('/v1/task',undefined,true);this.authenticated=true;this.authAt=this.clock();}
   catch {this.authenticated=false;}
  }
  // An exact journaled receipt resolves a lost response without sending again.
  const pending=this.session.pending;
  if(pending && (state.events||[]).some(e=>e.request_id===pending.request_id && e.visit_id===pending.visit_id && e.event===pending.event)){
   this.session.pending=null;this.save();
  }
  return state;
 }
 async register(){
  if(this.state?.worker_ready&&!this.owned)throw new Error('其他调用方在线，本页面只读');
  const result=await this.request('/v1/worker',{worker_id:'battery',instance_id:this.session.instance_id,point_ids:['point_1','point_2']});
  this.session.token=result.token;this.authenticated=true;this.authAt=this.clock();this.save();return result;
 }
 async heartbeat(){
  if(!this.session.token)return;
  try {const result=await this.request('/v1/heartbeat',{},true);this.authenticated=true;this.authAt=this.clock();return result;}
  catch(e){this.authenticated=false;throw e;}
 }
 async sendEvent(displayed,event,checks={},reason=''){
  if(!sameVisit(displayed,this.state?.task))throw new Error('到访已变化，请刷新后核对当前地点');
  const view=deriveView(this.state,this.owned,this.fresh,checks);
  const allowed={accept:view.canAccept,complete:view.canComplete,fail:view.canFail,cancelled:view.canCancel};
  if(!allowed[event])throw new Error('请确认作业状态、控制权归还及当前联调权限');
  if(event==='fail'&&!reason.trim())throw new Error('请填写实际失败原因');
  const body={...Object.fromEntries(['mission_id','visit_id','point_id'].map(k=>[k,displayed[k]])),event};
  if(event==='complete')Object.assign(body,{control_released:true,navigation_ready:true});
  if(event==='cancelled')body.control_released=true;
  if(event==='fail')body.error=reason.trim().slice(0,300);
  const old=this.session.pending;
  if(old){
   const payload={...old};delete payload.request_id;
   if(JSON.stringify(payload)!==JSON.stringify(body))throw new Error('上次回执尚待核对，请先重试同一回执');
  } else {this.session.pending={...body,request_id:this.uuid()};this.save();}
  return this.retryPending();
 }
 async retryPending(){
  const pending=this.session.pending;
  if(!pending)throw new Error('没有待重试回执');
  if(!this.owned||!this.fresh)throw new Error('联调连接已过期，请先刷新并核对调用身份');
  try {
   const result=await this.request('/v1/task',pending,true);
   this.session.pending=null;this.session.rejected=null;this.save();return result;
  } catch(error) {
   // A definite protocol rejection is different from uncertain transport/5xx.
   // Retain its identity for display, but do not lock out cancellation recovery.
   if([400,403,409,413,415].includes(error.status)){
    this.session.rejected={request_id:pending.request_id,visit_id:pending.visit_id,event:pending.event,status:error.status,error:error.message};
    this.session.pending=null;this.save();
   }
   throw error;
  }
 }
}

export function startDashboard(doc=globalThis.document){
 const client=new ManualClient(),by=id=>doc.getElementById(id);
 let displayed=null,busy=false,visit=null,stopped=false;
 const checks=()=>({work:by('confirm-work').checked,released:by('confirm-released').checked,ready:by('confirm-ready').checked,stopped:by('confirm-stopped').checked});
 const text=(id,value)=>{by(id).textContent=value;};
 function render(){
  const s=client.state,a=s?.task;
  if(a?.visit_id!==visit){visit=a?.visit_id;for(const id of ['work','released','ready','stopped'])by('confirm-'+id).checked=false;}
  const v=deriveView(s,client.owned,client.fresh,checks());
  displayed=a?{...a}:null;
  text('connection',client.fresh?'状态实时':'连接过期');by('connection').className='badge '+(client.fresh?'':'bad');
  text('updated',client.stateAt>0?'最后更新 '+new Date(client.stateAt).toLocaleTimeString('zh-CN'):'尚未收到状态');
  text('mode',v.mode);text('mode-detail',client.owned?'本页面仅发送交接回执，不执行地点作业。':s?.worker_ready?'真实调用方持有接口；本页不能代发完成信号。':'启用联调不会启动路线，等待导航真实到点。');
  text('point',a?({'point_1':'地点一 · point_1','point_2':'地点二 · point_2'}[a.point_id]||a.point_id):'暂无到点任务');
  text('phase',phases[a?.status]||'等待导航到点并停稳');text('notification',v.notification);text('work',v.work);text('navigation',v.navigation);
  text('worker',s?.worker_ready?(client.owned?'本页调用方在线':'其他调用方在线'):'调用方离线');
  text('release',a?.control_released?'已收到归还控制权声明，仍由导航核验':'尚未确认控制权归还');
  const gate=s?.navigation_check;
  text('check-reason',gate&&gate.visit_id===a?.visit_id?(Date.now()-gate.at_unix_ms<3000?gate.reason:'导航验收状态已过期，请检查导航进程'):'等待导航独立验收');
  text('mission-id',a?.mission_id||'—');text('visit-id',a?.visit_id||'—');text('route-map',a?`${a.route_point_id} / ${a.map_id}`:'—');
  text('arrival',Array.isArray(a?.arrival_pose)?a.arrival_pose.map(x=>Number(x).toFixed(3)).join(' / '):'—');
  by('error').hidden=!a?.error;text('error',a?.error||'');
  const steps=by('steps');steps.replaceChildren();for(const step of v.steps){const li=doc.createElement('li');li.textContent=step.label;li.className=step.done?'done':'';steps.append(li);}
  for(const [id,allowed] of Object.entries({register:v.canRegister,accept:v.canAccept,complete:v.canComplete,fail:v.canFail,cancelled:v.canCancel}))by(id).disabled=busy||!allowed||Boolean(client.session.pending);
  by('cancel-confirm').hidden=a?.status!=='cancelling';by('pending').hidden=!client.session.pending;by('retry').disabled=busy||!client.owned||!client.fresh;
  by('rejected').hidden=!client.session.rejected;text('rejected',client.session.rejected?'上次回执被拒绝，未据此推进交接：'+client.session.rejected.error:'');
  by('register').disabled=busy||!v.canRegister;
  const rows=by('events');rows.replaceChildren();const events=(s?.events||[]).slice().reverse();
  if(!events.length){const tr=doc.createElement('tr'),td=doc.createElement('td');td.colSpan=4;td.textContent='暂无交接记录';tr.append(td);rows.append(tr);}
  for(const event of events){const tr=doc.createElement('tr');for(const value of [new Date(event.at_unix_ms).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai'}),event.point_id||'—',labels[event.event]||event.event,event.error||(event.outcome==='cancelled'?'已取消，不继续旧目标':event.outcome==='completed'?'完成交接':phases[event.status]||event.status||'—')]){const td=doc.createElement('td');td.textContent=value;tr.append(td);}rows.append(tr);}
 }
 async function update(){try{await client.refresh();}catch(e){client.stateAt=-Infinity;text('message','状态连接失败：'+e.message);}render();}
 async function act(action){if(busy)return;busy=true;render();try{await action();text('message','信号已受理，请查看导航实际确认状态。');await update();}catch(e){text('message','未确认成功：'+e.message);}finally{busy=false;render();}}
 by('register').addEventListener('click',()=>act(()=>client.register()));
 by('accept').addEventListener('click',()=>{const target=displayed;act(()=>client.sendEvent(target,'accept'));});
 by('complete').addEventListener('click',()=>{const target=displayed,c=checks();act(()=>client.sendEvent(target,'complete',c));});
 by('fail').addEventListener('click',()=>{const target=displayed;act(()=>client.sendEvent(target,'fail',{},by('failure-reason').value));});
 by('cancelled').addEventListener('click',()=>{const target=displayed,c=checks();act(()=>client.sendEvent(target,'cancelled',c));});
 by('retry').addEventListener('click',()=>act(()=>client.retryPending()));by('refresh').addEventListener('click',()=>update());
 for(const id of ['work','released','ready','stopped'])by('confirm-'+id).addEventListener('change',render);
 async function poll(){if(stopped)return;await update();if(!stopped)setTimeout(poll,500);}
 async function beat(){if(stopped)return;try{await client.heartbeat();}catch{render();}if(!stopped)setTimeout(beat,200);}
 render();poll();beat();
 return {client,stop(){stopped=true;}};
}
if(typeof document!=='undefined')startDashboard(document);
