"""Partition identical per-device filtering into independent CPU workers."""
SOURCES=(('bottom','/single/lidar_raw'),('head','/unused_head/lidar'))
def selected_sources(value=''):
    if value not in ('','bottom','head'):raise ValueError('Invalid input filter source: '+value)
    return [item for item in SOURCES if not value or item[0]==value]
