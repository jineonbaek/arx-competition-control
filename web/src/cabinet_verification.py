"""Post-release RGB confirmation reusing grasp recognition and placement stability."""
import hashlib
import io
import json

import cv2
import numpy as np
from PIL import Image

from agent.grasp_pipeline import gpu_slot, segment
from agent.placement_backend import verify_stability


def verify_placement(preview, store, observation, token, check, sleep):
    capture = preview.captures['d435']
    check()
    frame = capture.capture(token=token)
    stream = io.BytesIO()
    # Recognition only: zero depth explicitly means unavailable, never measured geometry.
    np.savez_compressed(stream, rgb=frame['rgb'], raw_depth=np.zeros(frame['rgb'].shape[:2], np.uint16),
                        id=frame['id'], camera_id='d435', depth_status='unavailable')
    blob = stream.getvalue()
    image = io.BytesIO()
    Image.fromarray(frame['rgb']).save(image, format='JPEG', quality=95)
    manifest = dict(camera_id='d435', scene_sha256=hashlib.sha256(blob).hexdigest(),
                    cameras={'d435':dict(captured_at=frame['image_time'], geometry_ready=False)})
    ident = store.save_observation(manifest, {'d435':image.getvalue()}, geometry_archive=blob)['observation_id']
    output = store.path/'placement-verification'/ident
    output.mkdir(parents=True)
    with gpu_slot():
        report = segment(store, ident, output, lambda:store.cancelled)
    check()
    if report.get('status') != 'perception_accepted':
        raise ValueError('放置后未确认识别到电池')
    mask = np.asarray(Image.open(output/'mask.png').convert('L')) > 0
    plan = observation['plan']
    bounds = np.asarray(preview.battery['bounds_mm'])
    width, height, length = bounds[1]-bounds[0]
    axis, left, up = np.asarray(plan['R_base_from_tool']).T
    front = np.asarray(plan['leading_points_base_mm'][-1])
    corners = [front-axis*d+left*x+up*y for d in (0.,length)
               for x in (-width/2,width/2) for y in (-height/2,height/2)]
    pixels = preview._project(frame, corners)
    if any(p is None for p in pixels):
        raise ValueError('入仓目标不在D435视野中，不能确认放置')
    expected = np.zeros(mask.shape, np.uint8)
    cv2.fillConvexPoly(expected, cv2.convexHull(np.array(pixels,np.int32)), 1)
    if not mask.any() or np.mean(expected[mask]) < .8:
        raise ValueError('识别电池不在本次入仓目标位置')
    gray = cv2.cvtColor(frame['rgb'], cv2.COLOR_RGB2GRAY)
    points = cv2.goodFeaturesToTrack(gray, 100, .01, 7, mask=mask.astype(np.uint8)*255)
    if points is None or len(points) < 3:
        raise ValueError('电池可跟踪特征不足，未确认放置稳定')
    center = np.asarray(observation['window']['center_base_m'])*1000
    projected = preview._project(frame,[center,center+left*10,center+up*10])
    scale = np.column_stack((np.subtract(projected[1],projected[0]),
                             np.subtract(projected[2],projected[0])))/10
    inverse_scale = np.linalg.inv(scale)
    previous = dict(gray=gray, points=points, frame=frame, shift=np.zeros(2))

    def observe():
        check()
        fresh = capture.capture(token=token)
        if fresh['image_time'] <= previous['frame']['image_time']:
            raise ValueError('放置确认画面未更新')
        current = cv2.cvtColor(fresh['rgb'],cv2.COLOR_RGB2GRAY)
        tracked, status, _ = cv2.calcOpticalFlowPyrLK(previous['gray'],current,previous['points'],None)
        if tracked is None or status is None or np.count_nonzero(status) < 3:
            raise ValueError('放置后电池跟踪丢失')
        good = status.ravel().astype(bool)
        delta = np.median((tracked-previous['points'])[good,0,:],axis=0)
        previous['shift'] += delta
        previous.update(gray=current, points=tracked[good], frame=fresh)
        xy_mm = inverse_scale@previous['shift']
        return dict(image_time=fresh['image_time'],center_base_m=[*list(xy_mm/1000),0.])

    # GPU recognition time does not count toward the existing three-second stability check.
    return verify_stability(observe, check, sleep, reference=[0.,0.,0.])
