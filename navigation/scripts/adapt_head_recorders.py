#!/usr/bin/env python3
"""Make existing passive mapping recorders selectable by topic; preserve defaults."""
from pathlib import Path

root=Path(__file__).resolve().parents[1]
p=root/'scripts/record_manual_mapping.py'
s=p.read_text()
if '--odom-topic' not in s:
    assert 'a=p.parse_args();' in s
    s=s.replace("'/mapping_test/odometry'",'a.odom_topic').replace("'/mapping_test/map'",'a.map_topic')
    s=s.replace('a=p.parse_args();',"p.add_argument('--odom-topic',default='/mapping_test/odometry');p.add_argument('--map-topic',default='/mapping_test/map');a=p.parse_args();")
    p.write_text(s)
p=root/'scripts/record_mapping_bag_bounded.py'
s=p.read_text()
if '--topic-prefix' not in s:
    assert 'a=p.parse_args();' in s
    for suffix in ['lidar','imu','odometry']:
        s=s.replace(repr('/mapping_test/'+suffix),'a.topic_prefix+'+repr('/'+suffix))
    s=s.replace('a=p.parse_args();',"p.add_argument('--topic-prefix',default='/mapping_test');a=p.parse_args();")
    p.write_text(s)
