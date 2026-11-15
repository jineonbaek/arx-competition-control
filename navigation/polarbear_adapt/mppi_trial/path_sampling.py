"""Bound native MPPI path spacing without changing the audited planar geometry."""
import math

def sampled_route(route,spacing=.02,yaw_step=.04):
    if not 0<spacing<=.1 or not 0<yaw_step<=math.pi:raise ValueError('invalid sampling limits')
    points=[tuple(map(float,p)) for p in route]
    if not points or any(len(p)!=3 or not all(map(math.isfinite,p)) for p in points):raise ValueError('invalid path')
    output=[points[0]]
    for a,b in zip(points,points[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        yaw=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
        steps=max(1,math.ceil(math.hypot(dx,dy)/spacing),math.ceil(abs(yaw)/yaw_step))
        if len(output)+steps>20000:raise ValueError('path sampling limit exceeded')
        for step in range(1,steps):
            t=step/steps;output.append((a[0]+t*dx,a[1]+t*dy,a[2]+t*yaw))
        output.append(b)
    return output
