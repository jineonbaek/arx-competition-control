#!/usr/bin/env python3
"""Prepare a separate, fingerprinted SDK copy without constructor home/zero.

Never edits the vendor library. No SDK loading, ROS, CAN or device access.
Only supports the exact inspected x86-64 ELF; an SDK update must be reviewed.
"""
import argparse,hashlib,json,os,struct
from pathlib import Path

SOURCE_SHA256='d07d53d14da17f0a1db2753e3339f9e4c055dad72df613c811b9692c4bede85c'

def file_offset(data,address):
    if data[:6]!=b'\x7fELF\x02\x01' or struct.unpack_from('<H',data,18)[0]!=62:
        raise ValueError('Expected inspected little-endian x86-64 ELF')
    table=struct.unpack_from('<Q',data,32)[0]
    size,count=struct.unpack_from('<HH',data,54)
    for i in range(count):
        kind,flags,offset,virtual,physical,length=struct.unpack_from('<IIQQQQ',data,table+i*size)
        if kind==1 and flags&1 and virtual<=address<virtual+length:return offset+address-virtual
    raise ValueError('Startup address is outside executable ELF segments')

def patch_bytes(source):
    if hashlib.sha256(source).hexdigest()!=SOURCE_SHA256:
        raise ValueError('Unreviewed SDK fingerprint; no patch or startup permitted')
    result=bytearray(source)
    state=file_offset(source,0x150f18);gate=file_offset(source,0x150fcf)
    if source[state:state+10]!=bytes.fromhex('c78534ffffff01000000') or source[gate:gate+7]!=bytes.fromhex('0fb6851fffffff'):
        raise ValueError('SDK startup instructions do not match reviewed disassembly')
    # Retain original communication/error/enable/100-cycle protection setup.
    # Publish PROTECT (2), not GO_HOME (1), to the later SDK worker thread.
    result[state+6]=2
    # Preserve measured initial joints copied immediately before this gate.
    # Bypass the forced 3.55s home AND gripper zero/calibration block, landing
    # on the original completion print and callback/stack cleanup.
    result[gate:gate+7]=b'\xe9'+struct.pack('<i',0x151360-(0x150fcf+5))+b'\x90\x90'
    return bytes(result)

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--verify',action='store_true');a=p.parse_args()
    if a.source.resolve()==a.output.resolve():raise ValueError('Vendor SDK must remain unchanged')
    original=a.source.read_bytes();patched=patch_bytes(original)
    if a.verify:
        if not a.output.is_file() or a.output.read_bytes()!=patched:raise ValueError('Prepared no-home SDK is missing or changed')
    else:
        a.output.parent.mkdir(parents=True,exist_ok=True);temp=a.output.with_suffix('.so.tmp')
        temp.write_bytes(patched);temp.chmod(0o755);os.replace(temp,a.output)
        (a.output.parent/'manifest.json').write_text(json.dumps(dict(
            vendor_source=str(a.source.resolve()),source_sha256=SOURCE_SHA256,
            patched_sha256=hashlib.sha256(patched).hexdigest(),
            automatic_home=False,automatic_gripper_zero=False,
            protection_and_feedback_initialization=True),indent=2))
    print('Verified no-home startup SDK:',hashlib.sha256(patched).hexdigest())

if __name__=='__main__':main()
