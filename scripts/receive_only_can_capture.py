"""Finite SLCAN receive-only application. Adapter O opens CAN channel; no motor/control/data frames are transmitted."""
from pathlib import Path
import collections,concurrent.futures,json,re,subprocess,time
import serial
ROOT=Path('/home/arx/arx-competition-control')
PORTS={'left':'/dev/arxcan1','right':'/dev/arxcan3','platform':'/dev/arxcan5'}
ALLOWED={b'C\r',b'S8\r',b'O\r'}
def adapter_command(port,command,events):
    if command not in ALLOWED:raise ValueError('Only adapter close, bitrate and channel-open commands allowed; CAN data/remote frames forbidden')
    port.write(command);port.flush();time.sleep(0.15)
    response=port.read(65536)
    events.append({'adapter_command':command.decode().strip(),'response_hex':response.hex(),'time_ns':time.time_ns()})
    if b'\x07' in response:raise RuntimeError('Adapter rejected command; capture aborted')
    return response

def capture(item):
    role,path=item;events=[];frames=[];raw=[];error=None;confirmed=False
    try:
        owner=subprocess.run(['fuser',path],capture_output=True,text=True)
        if owner.returncode==0:raise RuntimeError('Serial device already in use: '+owner.stdout.strip())
        with serial.Serial(path,115200,timeout=0.05,write_timeout=0.5,exclusive=True) as port:
            try:
                initial=port.read(65536)
                if initial:events.append({'preexisting_bytes_hex':initial.hex()})
                adapter_command(port,b'C\r',events)
                adapter_command(port,b'S8\r',events)
                response=adapter_command(port,b'O\r',events)
                confirmed=b'\r' in response
                if not confirmed:events.append({'notice':'Adapter does not acknowledge channel-open; only serial reads follow, no CAN data/remote frames'})
                buffer=b'';end=time.monotonic()+12
                while time.monotonic()<end:
                    chunk=port.read(8192)
                    if not chunk:continue
                    timestamp=time.time_ns();raw.append({'time_ns':timestamp,'hex':chunk.hex()});buffer+=chunk
                    while b'\r' in buffer:
                        line,buffer=buffer.split(b'\r',1)
                        text=line.decode('ascii',errors='replace')
                        match=re.fullmatch(r't([0-9A-Fa-f]{3})([0-8])([0-9A-Fa-f]*)(?:)',text)
                        if match:
                            can_id,dlc,data=match.groups();n=int(dlc)
                            if len(data)==2*n:frames.append({'time_ns':timestamp,'can_id':int(can_id,16),'dlc':n,'data_hex':data.upper()})
                        elif text:events.append({'unparsed_line':text[:200]})
            finally:adapter_command(port,b'C\r',events)
    except Exception as exc:error=str(exc)
    result={'role':role,'port':path,'adapter_open_requested':True,'adapter_open_acknowledged':confirmed,'error':error,'adapter_events':events,'frame_count':len(frames),'can_id_counts':dict(collections.Counter(hex(f['can_id']) for f in frames)),'frames':frames,'raw_chunks':raw}
    print('CAPTURE',role,'adapter_open_ack',confirmed,'frames',len(frames),'error',error,flush=True)
    return result
if __name__=='__main__':
    started=time.time_ns()
    records=list(concurrent.futures.ThreadPoolExecutor(3).map(capture,PORTS.items()))
    report={'mode':'adapter_open_receive_only_application','can_transmit_frames':0,'sdk_hardware_objects_created':False,'duration_per_bus_s':12,'started_at_ns':started,'finished_at_ns':time.time_ns(),'buses':records}
    (ROOT/'reports/receive-only-can-capture.json').write_text(json.dumps(report,indent=2))
    print('SAVED',ROOT/'reports/receive-only-can-capture.json',flush=True)
