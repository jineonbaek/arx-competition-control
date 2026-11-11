"""Download only the first five frames and mesh from the official demo ZIP."""
import concurrent.futures
import hashlib
import io
import json
from pathlib import Path
import struct
import urllib.request
import zipfile
import zlib

URL="https://drive.usercontent.google.com/download?id=1AwV9sESDKMgXGUu2n1o0Pc4x2JGYdVB3&export=download&confirm=t"
ROOT=Path(__file__).resolve().parent/"demo_data"


def fetch(start,end):
    req=urllib.request.Request(URL,headers={"Range":f"bytes={start}-{end}"})
    with urllib.request.urlopen(req,timeout=60) as response:
        if response.status!=206:raise ValueError("Server did not honor Range")
        return response.read(),response.headers["Content-Range"]


class RemoteZip(io.RawIOBase):
    def __init__(self):
        self.size=int(fetch(0,0)[1].split("/")[-1])
        self.pos=0

    def seekable(self):return True
    def tell(self):return self.pos

    def seek(self,offset,whence=0):
        self.pos=offset if whence==0 else self.pos+offset if whence==1 else self.size+offset
        return self.pos

    def read(self,n=-1):
        n=self.size-self.pos if n<0 else min(n,self.size-self.pos)
        if n<=0:return b""
        data,_=fetch(self.pos,self.pos+n-1)
        self.pos+=len(data)
        return data


def extract(info):
    dest=ROOT/info.filename
    if not dest.resolve().is_relative_to(ROOT.resolve()):raise ValueError("Unsafe archive path")
    raw,_=fetch(info.header_offset,info.header_offset+info.compress_size+65565)
    if raw[:4]!=b"PK\x03\x04":raise ValueError("Invalid ZIP header")
    name_len,extra_len=struct.unpack_from("<HH",raw,26)
    start=30+name_len+extra_len
    packed=raw[start:start+info.compress_size]
    if info.compress_type==zipfile.ZIP_DEFLATED:data=zlib.decompress(packed,-15)
    elif info.compress_type==zipfile.ZIP_STORED:data=packed
    else:raise ValueError("Unsupported compression")
    if len(data)!=info.file_size or zlib.crc32(data)!=info.CRC:raise ValueError("ZIP CRC mismatch")
    dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_bytes(data)
    return dict(path=info.filename,bytes=len(data),sha256=hashlib.sha256(data).hexdigest())


def main():
    ROOT.mkdir(exist_ok=True)
    with zipfile.ZipFile(RemoteZip()) as archive:
        names=archive.namelist()
        rgb=sorted(n for n in names if n.startswith("mustard0/rgb/") and n.endswith(".png"))[:5]
        targets=set(rgb+[n.replace("/rgb/","/depth/") for n in rgb])
        targets.update(n for n in names if n.startswith(("mustard0/mesh/","mustard0/masks/")) and not n.endswith("/"))
        targets.add("mustard0/cam_K.txt")
        infos=[archive.getinfo(n) for n in sorted(targets)]
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        files=list(pool.map(extract,infos))
    (ROOT/"sources.json").write_text(json.dumps(dict(url=URL,subset_frames=5,files=files),indent=2))
    print(f"Official demo subset: {len(files)} files, {sum(x['bytes'] for x in files)} bytes")


if __name__=="__main__":main()
