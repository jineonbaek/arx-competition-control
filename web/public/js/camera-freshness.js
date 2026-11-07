export function frameFresh(meta,serverNowMs) {
 if(meta?.state!=='online' || !Number.isFinite(meta.captured_at) || !Number.isFinite(serverNowMs))return false;
 const age=serverNowMs-meta.captured_at*1000;
 return age>=-1500 && age<=3000;
}

export function cameraImagePath(key,meta){
 if(!['d435','d405','astra'].includes(key))throw new Error('Unknown camera');
 const filename=meta?.image_file;
 const pattern=new RegExp('^'+key+'-[vb]-[a-f0-9]+-[0-9]+\\.jpg$');
 return typeof filename==='string' && pattern.test(filename) ? '/camera-live/'+filename : '/camera-live/'+key+'.jpg';
}
export function chooseCameraFrame(key,raw,boxed,serverNowMs){
 return boxed?.source_camera===key && boxed.whole_battery && frameFresh(boxed,serverNowMs) ? boxed : raw;
}

export function depthButtonView(status){
 if(!status)return {label:'开启深度',disabled:true,pressed:false,hint:'深度状态不可用'};
 const hints={disabled:'深度已关闭 · 仅彩色预览',starting:'正在开启深度，等待真实深度帧',stopping:'正在停止深度，等待采集端确认',online:'深度采集中',unavailable:'采集状态过期或服务未运行',error:'深度采集出错'};
 return {label:status.requested?'关闭深度':'开启深度',
  disabled:!status.requested && (!status.color_online || status.state==='stopping'),
  pressed:status.active===true,hint:(hints[status.state] || '深度状态未知')+(status.message?'：'+status.message:'')};
}
