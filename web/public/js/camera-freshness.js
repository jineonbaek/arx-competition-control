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
