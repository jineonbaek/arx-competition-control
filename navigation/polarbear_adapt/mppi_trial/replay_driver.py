"""Compile and replay original/candidate state machines; never imports ROS or SDK."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('record', type=Path)
parser.add_argument('--original-headers', required=True, type=Path)
parser.add_argument('--out', required=True, type=Path)
args = parser.parse_args()
root = Path(__file__).resolve().parent
record = json.loads(args.record.read_text())
rows = record['samples']
payload = ''.join(' '.join(map(str, [s[0], *s[4:7]]))+'\n' for s in rows)
variants = ['all_axes', 'yaw_zero_diagnostic', 'lateral_zero_diagnostic',
            'yaw_and_lateral_zero_diagnostic']
report = {
    'model': '400Hz replay; ready=true; request receipt approximated by sample timestamp',
    'limitations': 'Not measured CAN output; excludes HTTP/DDS delays and other command sources',
    'record_sha256': hashlib.sha256(args.record.read_bytes()).hexdigest(),
    'user_measurement': 'SDK 0.3 can move slowly; no SI speed or startup threshold measured',
}
with tempfile.TemporaryDirectory(prefix='mppi-driver-replay-') as work:
    for name, includes in [
        ('original', [args.original_headers]),
        ('candidate', [root/'hardware_candidate', args.original_headers]),
    ]:
        binary = Path(work)/name
        subprocess.run(['c++', '-std=c++17', '-O2',
                        *(f'-I{p.resolve()}' for p in includes),
                        str(root/'replay_chassis.cpp'), '-o', str(binary)], check=True)
        output = subprocess.check_output([str(binary)], input=payload, text=True)
        report[name] = {}
        for line in output.splitlines():
            variant, peak, mean, above, zero, below, active = map(float, line.split())
            report[name][variants[int(variant)]] = {
                'peak_backward_sdk': peak, 'mean_backward_sdk': mean,
                'seconds_at_or_above_legacy_0_14': above,
                'zero_output_seconds': zero, 'active_seconds': active,
                'below_legacy_0_14_seconds': below,
            }
args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
for name in ('original', 'candidate'):
    print(name, json.dumps(report[name]['all_axes']))
