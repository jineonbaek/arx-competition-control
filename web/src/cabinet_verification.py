"""Post-release RGB confirmation reusing grasp recognition and placement stability."""
import hashlib
import io

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
    # Track the recognized patch directly, including batteries with no corner features.
    y, x = np.nonzero(mask)
    if not len(x):
        raise ValueError('放置后未确认识别到电池')
    x0, y0, x1, y1 = int(x.min()), int(y.min()), int(x.max()+1), int(y.max()+1)
    template = frame['rgb'][y0:y1,x0:x1].copy()
    template_mask = mask[y0:y1,x0:x1].astype(np.uint8)*255
    _, left, up = np.asarray(plan['R_base_from_tool']).T
    center = np.asarray(observation['window']['center_base_m'])*1000
    projected = preview._project(frame,[center,center+left*10,center+up*10])
    scale = np.column_stack((np.subtract(projected[1],projected[0]),
                             np.subtract(projected[2],projected[0])))/10
    inverse_scale = np.linalg.inv(scale)
    previous = dict(frame=frame)

    def observe():
        check()
        fresh = capture.capture(token=token)
        if fresh['image_time'] <= previous['frame']['image_time']:
            raise ValueError('放置确认画面未更新')
        matched = cv2.matchTemplate(fresh['rgb'], template, cv2.TM_SQDIFF, mask=template_mask)
        _, _, location, _ = cv2.minMaxLoc(matched)
        previous['frame'] = fresh
        xy_mm = inverse_scale@(np.asarray(location)-[x0,y0])
        return dict(image_time=fresh['image_time'],center_base_m=[*list(xy_mm/1000),0.])

    # GPU recognition time does not count toward the existing three-second stability check.
    return verify_stability(observe, check, sleep, reference=[0.,0.,0.])
