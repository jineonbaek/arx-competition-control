#!/usr/bin/env python3
"""Install camera-only dependencies privately; no USB reset or robot access."""
import hashlib
import fcntl
from pathlib import Path
import subprocess
import sys
import urllib.request

ROOT=Path(__file__).resolve().parents[2]
REVISION="210f530144314da7c70072215bacba12130d9b20"
BASE=f"https://raw.githubusercontent.com/orbbec/ros_astra_camera/{REVISION}/include/openni2_redist/x64/"
FILES={
 "libOpenNI2.so":"29e9cafea8fec360edf50f88e2afc5b4b793664d28b6d91e144095893f0efe17",
 "OpenNI.ini":"3896e8f831c6a56576e9e93a2e5b3e1d3fa72041e7d01e1d7f23b7cd907e7eff",
 "OpenNI2/Drivers/liborbbec.so":"116aade45a425af1dbbf50c1afec227c415594b78a187bf9bd2c2254c56fee3e",
 "OpenNI2/Drivers/orbbec.ini":"bd8f6a89a7d9e659f2f6b998393b145eca7d33d1e499dd036ba914cd0dacce32",
 "OpenNI2/Drivers/libOniFile.so":"22f17086e5058904f5df76e7cffca9f37a914286f50467e76e78b0f62669cbab",
 "OpenNI2/Drivers/OniFile.ini":"f50a20ca76350f8e5e81b50477714f9a1b1db9b49047f5089b90644526ba9505",
}


def patch_binding(path):
    source=path.read_text()
    before="OniDeviceInfo._fields_ = [\n    ('uri', (ctypes.c_char * 256)),\n    ('vendor', (ctypes.c_char * 256)),\n    ('name', (ctypes.c_char * 256)),\n"
    after=before+"    ('serialNumber', (ctypes.c_char * 256)),\n"
    # Orbbec adds serialNumber to OniDeviceInfo; stock openni 2.3.0 writes past its buffer.
    if after in source:return
    if source.count(before)!=1:raise RuntimeError("Unexpected OpenNI binding; refusing an unsafe ABI patch")
    path.write_text(source.replace(before,after))


def main():
    if sys.platform!="linux":raise SystemExit("Run with the ROG system Python on Linux x64")
    import platform
    if platform.machine()!="x86_64":raise SystemExit("Only the pinned Linux x64 SDK is supported")
    logs=ROOT/"web/logs";logs.mkdir(parents=True,exist_ok=True)
    with (logs/"camera-preview.lock").open("a+") as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit("Stop the camera preview owner before replacing camera SDK libraries")
        subprocess.run([sys.executable,"-m","pip","install","--no-deps","--upgrade","--target",str(ROOT/"vision/vendor-python"),"-r",str(ROOT/"vision/requirements-rgbd.txt")],check=True)
        patch_binding(ROOT/"vision/vendor-python/openni/_openni2.py")
        for name,expected in FILES.items():
            data=urllib.request.urlopen(BASE+name,timeout=30).read()
            if hashlib.sha256(data).hexdigest()!=expected:raise RuntimeError("SDK checksum mismatch: "+name)
            target=ROOT/"vision/vendor-openni"/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
        print("Camera dependencies installed; install the scoped Astra udev rule as documented.")

if __name__=="__main__":main()
