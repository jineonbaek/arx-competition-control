"""Restore obstacle evidence during a supervised restart within one LIO session."""
from pathlib import Path

import numpy as np


def verify_session(archive):
    pid = int(archive['lio_pid'])
    proc = Path('/proc') / str(pid)
    ticks = int((proc / 'stat').read_text().rsplit(')', 1)[1].split()[19])
    if (Path('/proc/sys/kernel/random/boot_id').read_text().strip() != str(archive['boot'])
            or ticks != int(archive['lio_ticks'])
            or b'fastlio_mapping' not in (proc / 'cmdline').read_bytes()):
        raise ValueError('Obstacle checkpoint belongs to a different LIO session')


def load_history(world, path):
    with np.load(path, allow_pickle=False) as data:
        verify_session(data)
        keys, low, high, times = (data[k] for k in ('keys', 'low', 'high', 'ns'))
        if (keys.shape != low.shape or low.shape != high.shape
                or keys.ndim != 2 or keys.shape[1] != 3 or len(times) != len(keys)
                or not np.isfinite([low, high]).all() or np.any(low > high)):
            raise ValueError('Invalid historical obstacle bounds')
        for k, a, b, ns in zip(keys, low, high, times):
            key = tuple(k)
            world.cells[key] = (a[2], b[2], int(ns))
            world.bounds[key] = (a.copy(), b.copy())
            world.columns[key[:2]] += 1
            world.tiles.setdefault((key[0] // 10, key[1] // 10), set()).add(key)
        if world.index is not None and len(keys):
            world.index.set(keys, np.column_stack((low[:, 2], high[:, 2])))
        world.cleared_xy.update(map(tuple, data['cleared_xy']))
        return len(world.cells)


def restore_map_clears(node, path):
    with np.load(path, allow_pickle=False) as data:
        verify_session(data)
        if abs(float(data['map_resolution']) - node.res) > 1e-7:
            raise ValueError('Static map resolution changed during restart')
        # Carry confirmed changes at map cell centres, even if SLAM expands the map.
        coords = np.floor((data['static_xy'] - node.anchor) / node.res).astype(np.int64)
        node.cleared.update((coords[:, 0] << 32) | (coords[:, 1] & 0xffffffff))


def capture_static_clears(directory):
    """Snapshot measured base-map corrections independently of the planning overlay."""
    import time
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile
    rclpy.init()
    node = rclpy.create_node('export_confirmed_map_corrections')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps = {}
    try:
        for key in ('reference_map', 'persistent_map'):
            node.create_subscription(OccupancyGrid, '/pb/' + key,
                                     lambda m, k=key: maps.update({k: m}), qos)
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if len(maps) != 2:
                continue
            base, corrected = maps['reference_map'], maps['persistent_map']
            if base.info != corrected.info or base.header.frame_id != corrected.header.frame_id:
                continue
            cells = np.flatnonzero((np.asarray(base.data) == 100) & (np.asarray(corrected.data) == 0))
            width = base.info.width
            res = base.info.resolution
            origin = base.info.origin.position
            xy = (np.c_[cells % width, cells // width] + .5) * res + [origin.x, origin.y]
            np.savez(directory / 'static-clears.npz', xy=xy, res=res)
            return len(cells)
        raise RuntimeError('No matching reference and persistent maps for checkpoint')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    print('retained confirmed static map corrections', capture_static_clears(Path(sys.argv[1])))
