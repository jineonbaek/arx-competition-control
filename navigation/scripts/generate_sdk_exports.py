#!/usr/bin/env python3
"""Expose public C SDK API only; isolate bundled spdlog from ROS spdlog."""
from pathlib import Path
import re
root=Path(__file__).resolve().parents[1]
text=(root/'deps/Livox-SDK2/include/livox_lidar_api.h').read_text()
text=re.sub(r'/\*.*?\*/','',text,flags=re.S)
names=sorted(set(re.findall(r'^(?:void|bool|livox_status|int|uint\d+_t)\s+(\w+)\s*\(',text,re.M)))
if 'LivoxLidarSdkInit' not in names or len(names)<20:raise SystemExit('Unexpected SDK API; export map not generated')
(root/'config').mkdir(exist_ok=True)
(root/'config/livox_sdk_exports.map').write_text('{\n  global:\n'+''.join('    '+name+';\n' for name in names)+'  local: *;\n};\n')
print(f'Generated export map for {len(names)} public SDK C functions')
