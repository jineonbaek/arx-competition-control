"""PlanD camera-space battery face detector; never produces motion targets."""
import base64
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np

def _ordered_quad(points):
    points = np.asarray(points, np.float32)
    center = points.mean(0)
    points = points[np.argsort(np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0]))]
    return np.roll(points, -int(np.argmin(points.sum(1))), axis=0)


def battery_upper_face(rgb, xyz):
    """Detect the user-approved top/right, camera-facing battery plane without a pose prior."""
    rgb = np.asarray(rgb, np.uint8); xyz = np.asarray(xyz, float)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or xyz.shape != (*rgb.shape[:2], 3):
        return None
    height, width = rgb.shape[:2]
    depth = xyz[:, :, 2]
    roi = np.zeros((height, width), bool)
    roi_bottom = max(1, int(round(height*.55)))
    roi_left = int(round(width*.46))
    roi[:roi_bottom, roi_left:] = True
    valid = roi & np.isfinite(xyz).all(axis=2) & (depth >= .15) & (depth <= 1.2)
    ys, xs = np.nonzero(valid)
    minimum = max(300, int(width*height*.003))
    if len(xs) < minimum:
        return None
    points = xyz[ys, xs]
    rng = np.random.default_rng(29)
    remaining = np.arange(len(points))
    planes = []
    for _ in range(8):
        if len(remaining) < minimum:
            break
        active = points[remaining]
        sample = active if len(active) <= 16000 else active[rng.choice(len(active), 16000, replace=False)]
        best = None
        for _ in range(260):
            a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
            normal = np.cross(b-a, c-a); length = np.linalg.norm(normal)
            if length < 1e-8:
                continue
            normal /= length
            inliers = np.abs((active-a) @ normal) <= .005
            count = int(inliers.sum())
            if best is None or count > best[0]:
                best = count, normal, a, inliers
        if best is None or best[0] < minimum:
            break
        _, normal, anchor, inliers = best
        planes.append((normal, anchor))
        remaining = remaining[~inliers]

    candidates = []
    minimum_area = max(500, int(width*height*.004))
    close_kernel = np.ones((max(5, int(round(height*.065))),
                            max(3, int(round(width*.014)))), np.uint8)
    for normal, anchor in planes:
        facing = abs(float(normal[2]))
        if facing < .90:
            continue
        inliers = np.abs((points-anchor) @ normal) <= .008
        raw_mask = np.zeros((height, width), np.uint8)
        raw_mask[ys[inliers], xs[inliers]] = 1
        closed_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_CLOSE, close_kernel)
        count, labels, stats, centers = cv2.connectedComponentsWithStats(closed_mask, 8)
        for index in range(1, count):
            area = int(stats[index, cv2.CC_STAT_AREA])
            left, top, box_width, box_height = map(int, stats[index, :4])
            if (area < minimum_area or
                    box_width < max(28, int(width*.10)) or box_height < max(20, int(height*.08))):
                continue
            right = left + box_width - 1
            bottom = top + box_height - 1
            aspect = box_width / float(box_height)
            if not .40 <= aspect <= 5.0:
                continue
            if top > max(2, int(height*.03)) or right < width-2 or centers[index, 0] < width*.60:
                continue
            # A background filling the search ROI inherits its artificial left and
            # bottom borders. The approved battery has measured object boundaries
            # strictly inside both of those ROI edges.
            if (left <= roi_left + max(3, int(width*.02)) or
                    bottom >= roi_bottom-2):
                continue
            component = labels == index
            support = component & (raw_mask > 0) & np.isfinite(depth)
            support_count = int(support.sum())
            support_ratio = support_count / float(area)
            if support_count < minimum_area or support_ratio < .20:
                continue
            patch_width = max(4, int(round(box_width*.18)))
            patch_height = max(4, int(round(box_height*.18)))
            corner_patches = (
                support[top:top+patch_height, left:left+patch_width],
                support[top:top+patch_height, right-patch_width+1:right+1],
                support[bottom-patch_height+1:bottom+1, left:left+patch_width],
                support[bottom-patch_height+1:bottom+1, right-patch_width+1:right+1],
            )
            if any(np.count_nonzero(patch) < max(3, int(patch.size*.02)) for patch in corner_patches):
                continue
            measured_depth = depth[support]
            if not len(measured_depth) or not np.isfinite(measured_depth).all():
                continue
            median_depth = float(np.median(measured_depth))
            area_factor = min(2., support_count/float(minimum_area))
            score = area_factor*facing*(.5+.5*support_ratio)/(median_depth+.1)
            candidates.append((score, area, support_count, support_ratio, facing,
                               median_depth, component, normal, anchor))
    if not candidates:
        return None
    _, area, support_count, support_ratio, facing, median_depth, component, normal, anchor = max(candidates, key=lambda row: row[0])
    contour_set, _ = cv2.findContours(component.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contour_set:
        return None
    contour = max(contour_set, key=cv2.contourArea)
    hull = cv2.convexHull(contour)
    quad = cv2.boxPoints(cv2.minAreaRect(hull))
    quad[:, 0] = np.clip(quad[:, 0], 0, width-1)
    quad[:, 1] = np.clip(quad[:, 1], 0, height-1)
    quad = _ordered_quad(quad)
    if cv2.contourArea(quad.astype(np.float32)) < max(900., width*height*.02):
        return None
    contacts = []
    if float(quad[:, 1].min()) <= 2: contacts.append('top')
    if float(quad[:, 0].max()) >= width-2: contacts.append('right')
    rect_area = max(1., float(cv2.contourArea(quad.astype(np.float32))))
    visible_fraction = float(np.clip(support_count/rect_area, 0., 1.))
    confidence = float(np.clip(.45*facing + .35*visible_fraction +
                               .20*min(1., support_count/(minimum_area*2.)), 0., 1.))
    return dict(method='upper_front_depth_plane', quad=quad, contour=quad.copy(),
                median_depth_m=median_depth, plane_normal=np.asarray(normal, float),
                plane_anchor=np.asarray(anchor, float), boundary_contacts=contacts,
                visible_fraction=visible_fraction, confidence=confidence,
                insertion_axis_available=False, hidden_space_clearance_proven=False)


class PlanDVision:
    def __init__(self, capture, controller):
        self.capture = capture
        self.controller = controller
        self.lock = threading.Lock()
        self.state = dict(status='idle', running=False, preview_only=True,
                          reason='等待采集 PlanD 电池面', image=None)

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def refresh(self):
        with self.lock:
            if self.state['running']:
                return dict(self.state)
            state = self.controller.snapshot()
            if state.get('motion_reserved') or state.get('competition_reserved') or state.get('chassis_active'):
                raise ValueError('请等待当前任务结束后采集 PlanD 画面')
            self.state = dict(status='capturing', running=True, preview_only=True,
                              reason='正在采集 D435 同帧彩色与深度', image=None)
        threading.Thread(target=self._run, daemon=True, name='plan-d-preview').start()
        return self.snapshot()

    def _run(self):
        started = time.time()
        try:
            view = self.capture.action({'op': 'capture'})
            with np.load(self.capture.cache/'frame.npz', allow_pickle=False) as frame:
                ident = str(frame['id'].item())
                captured = float(frame['image_time'].item())
                if ident != view['id'] or captured < started or not -.05 <= time.time()-captured < 2.:
                    raise ValueError('采集画面已更新或过期，请重新采集')
                rgb, xyz = frame['rgb'].copy(), frame['xyz'].copy()
            observation = battery_upper_face(rgb, xyz)
            canvas = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            result = dict(status='not_found', preview_only=True, frame_id=ident,
                          captured_at=captured, insertion_axis_available=False,
                          reason='本轮未找到符合 PlanD 条件的电池面', metrics=None)
            if observation is not None:
                quad = np.asarray(observation['quad'])
                cv2.polylines(canvas, [np.rint(quad).astype(np.int32)], True, (255,0,255), 3)
                result.update(status='detected', reason='已识别电池面；等待把手定位后才能生成抓取点',
                              metrics=dict(quad_px=quad.tolist(), depth_m=float(observation['median_depth_m']),
                                           normal=np.asarray(observation['plane_normal']).tolist(),
                                           confidence=float(observation['confidence']),
                                           boundary_contacts=observation['boundary_contacts']))
            ok, jpeg = cv2.imencode('.jpg', canvas)
            if not ok:
                raise ValueError('PlanD 识别图编码失败')
            result['image'] = 'data:image/jpeg;base64,'+base64.b64encode(jpeg).decode('ascii')
            result['running'] = False
        except Exception as error:
            result = dict(status='unavailable', running=False, preview_only=True,
                          reason=str(error), image=None, metrics=None)
        with self.lock:
            self.state = result
