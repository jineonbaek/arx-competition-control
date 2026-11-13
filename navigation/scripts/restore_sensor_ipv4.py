#!/usr/bin/env python3
"""Restore only the temporary Livox receive IP after an ARP conflict probe.

Run via sudo -n python3. Does not modify the primary address or NetworkManager.
"""
import json
from pathlib import Path
import socket
import struct
import subprocess
import time

interface, target = 'enp130s0', '192.168.1.50'
addresses = json.loads(subprocess.check_output(['ip', '-j', 'addr', 'show', 'dev', interface]))
if any(a.get('local') == target for a in addresses[0]['addr_info']):
    print('Sensor receive IPv4 already present')
    raise SystemExit(0)
mac = bytes.fromhex((Path('/sys/class/net')/interface/'address').read_text().strip().replace(':', ''))
ip = socket.inet_aton(target)
packet = b'\xff'*6+mac+b'\x08\x06'+struct.pack('!HHBBH', 1, 0x0800, 6, 4, 1)+mac+b'\x00'*4+b'\x00'*6+ip
with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0806)) as s:
    s.bind((interface, 0))
    s.settimeout(.1)
    for _ in range(3):
        s.send(packet)
        end = time.monotonic()+.4
        while time.monotonic() < end:
            try:
                response = s.recv(2048)
            except socket.timeout:
                continue
            if len(response) >= 42 and response[12:14] == b'\x08\x06' and response[22:28] != mac:
                if response[28:32] == ip or (response[28:32] == b'\0'*4 and response[38:42] == ip):
                    raise SystemExit('ARP conflict detected; receive IPv4 was not added')
subprocess.run(['ip', 'addr', 'add', target+'/24', 'dev', interface], check=True)
print('Restored temporary receive IPv4; primary address unchanged')
