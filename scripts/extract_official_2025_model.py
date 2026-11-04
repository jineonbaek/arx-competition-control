from pathlib import Path
import urllib.request,ctypes as c,ctypes.util,json,hashlib
r=Path('/home/arx/arx-competition-control/references/calibration-followup');r.mkdir(exist_ok=True)
u='https://raw.githubusercontent.com/ARXroboticsX/ARX_Model/7bf64651dd535065a2b9f0f1176e82755e3c1734/X5/X5-2025.7z'
p=r/'X5-2025.7z'; b=urllib.request.urlopen(u,timeout=30).read(20000000);assert len(b)<20000000;p.write_bytes(b)
a=c.CDLL(ctypes.util.find_library('archive'))
a.archive_read_new.restype=c.c_void_p
for name,args,ret in [('archive_read_support_filter_all',[c.c_void_p],c.c_int),('archive_read_support_format_all',[c.c_void_p],c.c_int),('archive_read_open_filename',[c.c_void_p,c.c_char_p,c.c_size_t],c.c_int),('archive_read_next_header',[c.c_void_p,c.POINTER(c.c_void_p)],c.c_int),('archive_entry_pathname',[c.c_void_p],c.c_char_p),('archive_read_data',[c.c_void_p,c.c_void_p,c.c_size_t],c.c_ssize_t),('archive_read_data_skip',[c.c_void_p],c.c_int),('archive_read_free',[c.c_void_p],c.c_int)]:
 f=getattr(a,name);f.argtypes=args;f.restype=ret
h=a.archive_read_new();a.archive_read_support_filter_all(h);a.archive_read_support_format_all(h);assert a.archive_read_open_filename(h,str(p).encode(),10240)==0
entry=c.c_void_p();names=[];saved=[]
while a.archive_read_next_header(h,c.byref(entry))==0:
 name=a.archive_entry_pathname(entry).decode();names.append(name)
 if name.lower().endswith(('.urdf','.xacro','.yaml','.yml','.xml','.txt')):
  buf=c.create_string_buffer(65536);chunks=[]
  while True:
   n=a.archive_read_data(h,buf,len(buf))
   if n<=0:break
   chunks.append(buf.raw[:n])
  data=b''.join(chunks);dest=r/('model2025-'+Path(name).name);dest.write_bytes(data);saved.append(str(dest));print('SAVED',name,len(data))
 else:a.archive_read_data_skip(h)
a.archive_read_free(h)
(r/'official-2025-model-manifest.json').write_text(json.dumps({'url':u,'sha256':hashlib.sha256(b).hexdigest(),'entries':names,'saved':saved},ensure_ascii=False,indent=2))
print('ARCHIVE',len(b),'entries',len(names))
