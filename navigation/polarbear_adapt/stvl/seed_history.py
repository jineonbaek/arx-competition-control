"""Rebin measured historical return bounds into STVL's map grid, retaining acquisition times."""
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def write_seed(path, low, high, ns, rotation, translation, size=float(np.float32(.05))):
    middle=(low+high)*.5@rotation.T+translation
    half=(high-low)*.5@np.abs(rotation.T)
    zlo,zhi=(middle-half)[:,2],(middle+half)[:,2]
    first=np.floor((middle-half)/size).astype(int)
    last=np.maximum(first,np.floor(np.nextafter(middle+half,-np.inf)/size).astype(int))
    rows={}
    for a,b,lo,hi,stamp in zip(first,last,zlo,zhi,ns):
        for x in range(a[0],b[0]+1):
            for y in range(a[1],b[1]+1):
                for z in range(a[2],b[2]+1):
                    key=(x,y,z)
                    zl=max(z*size,lo);zh=min((z+1)*size,hi)
                    newest=int(stamp)
                    if key in rows:
                        old=rows[key];zl=min(zl,old[0]);zh=max(zh,old[1]);newest=max(old[2],newest)
                    rows[key]=(zl,zh,newest)
    out=Path(path)
    with out.with_suffix('.tmp').open('w') as f:
        for (x,y,z),(zl,zh,t) in rows.items():
            f.write(f'{x} {y} {z} {zl:.9f} {zh:.9f} {t}\n')
    out.with_suffix('.tmp').replace(out)
    return len(rows)


def current_transform():
    import time,rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer,TransformListener,TransformException
    rclpy.init();node=rclpy.create_node('capture_stvl_seed_transform')
    buf=Buffer();listener=TransformListener(buf,node)
    end=time.monotonic()+6
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            try:tf=buf.lookup_transform('map','odom',Time())
            except TransformException:continue
            if abs((node.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds)>500_000_000:continue
            t=tf.transform.translation;q=tf.transform.rotation
            return Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),[t.x,t.y,t.z]
        raise RuntimeError('No recent map-from-odom transform')
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    cli=argparse.ArgumentParser();cli.add_argument('checkpoint');cli.add_argument('output');args=cli.parse_args()
    with np.load(args.checkpoint,allow_pickle=False) as h:
        print('STVL seed rows:',write_seed(args.output,h['low'],h['high'],h['ns'],*current_transform()))
