"""Apply per-device receive switches before Livox delivers packets to any ROS topic."""
import json
import os
import subprocess
import sys
from pathlib import Path

CONFIG=Path(__file__).resolve().parents[1]/'config'
STORED=CONFIG/'lidar_streams.json'


def state():
    if STORED.exists():
        data=json.loads(STORED.read_text())
        if any(type(data.get(k)) is not bool for k in ('bottom','head')):
            raise ValueError('Invalid lidar switch settings')
        return data
    head=CONFIG/'head_obstacles.enabled'
    return dict(bottom=True,head=not head.exists() or head.read_text().strip()!='0')


def rules(bottom,head):
    table='inet arx_lidar_inputs'
    commands=[f'add table {table}',f'delete table {table}',f'add table {table}',
              f'add chain {table} ingress {{ type filter hook input priority -10; policy accept; }}']
    prefix=f'add rule {table} ingress iifname "enp130s0" ip daddr 192.168.1.50 ip saddr'
    if not bottom:
        commands.append(f'{prefix} 192.168.1.3 udp dport 56301 counter drop')
    if not head:
        commands.append(f'{prefix} 192.168.1.164 udp dport {{ 56301, 56401 }} counter drop')
    return '\n'.join(commands)+'\n'


def apply(data):
    subprocess.run(['sudo','-n','/usr/sbin/nft','-f','-'],input=rules(**data),text=True,check=True)


def main():
    data=state()
    if sys.argv[1]=='status':
        print(int(data['bottom']),int(data['head']))
    elif sys.argv[1]=='apply':
        apply(data)
    elif sys.argv[1]=='set' and len(sys.argv)==4 and all(x in ('0','1') for x in sys.argv[2:]):
        after=dict(bottom=sys.argv[2]=='1',head=sys.argv[3]=='1')
        tmp=STORED.with_suffix(f'.{os.getpid()}.tmp')
        tmp.write_text(json.dumps(after)+'\n')
        try:
            apply(after)
            tmp.replace(STORED)
        except BaseException:
            apply(data)
            raise
        finally:
            tmp.unlink(missing_ok=True)
    else:
        raise SystemExit('usage: lidar_ingress.py status | apply | set <bottom:0|1> <head:0|1>')


if __name__=='__main__':main()
