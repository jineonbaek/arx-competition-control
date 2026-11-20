"""Read-only D435 overlay for the held battery and cabinet opening."""
import json
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from agent.cabinet_preview_geometry import detect
from handeye_column import ColumnModel

ROOT = Path(__file__).resolve().parents[2]


def project_base(point, base_from_color, intrinsics):
    point = np.asarray(point, float)
    color_from_base = np.linalg.inv(np.asarray(base_from_color, float))
    camera = color_from_base[:3, :3] @ point + color_from_base[:3, 3]
    if camera[2] <= .03 or not np.isfinite(camera).all():
        return None
    fx = float(intrinsics.get('fx', intrinsics.get('focal_length_x')))
    fy = float(intrinsics.get('fy', intrinsics.get('focal_length_y')))
    cx = float(intrinsics.get('cx', intrinsics.get('ppx', intrinsics.get('principal_point_x'))))
    cy = float(intrinsics.get('cy', intrinsics.get('ppy', intrinsics.get('principal_point_y'))))
    return int(round(fx * camera[0] / camera[2] + cx)), int(round(fy * camera[1] / camera[2] + cy))


def tracked_object_pose(model, tcp, current_joints, current_lift,
                        reference_object, reference_joints, reference_lift):
    reference_tool = model.wrist_transform(reference_joints, reference_lift) @ tcp
    attachment = np.linalg.inv(reference_tool) @ np.asarray(reference_object, float)
    return model.wrist_transform(current_joints, current_lift) @ tcp @ attachment


def cabinet_axis_from_quad(quad, intrinsics, size_mm=(169.6, 79.0)):
    """Estimate the into-cabinet axis from the calibrated rectangular red rim."""
    quad = np.asarray(quad, np.float32)
    if quad.shape != (4, 2) or not np.isfinite(quad).all():
        return None
    width, height = np.asarray(size_mm, float) / 1000.
    model = np.asarray([[-width/2, -height/2, 0], [width/2, -height/2, 0],
                        [width/2, height/2, 0], [-width/2, height/2, 0]], np.float32)
    k = np.asarray([[intrinsics['fx'], 0, intrinsics.get('cx', intrinsics.get('ppx'))],
                    [0, intrinsics['fy'], intrinsics.get('cy', intrinsics.get('ppy'))], [0, 0, 1]], float)
    ok, rotation, translation = cv2.solvePnP(model, quad, k, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or translation[2, 0] <= .03:
        return None
    normal = cv2.Rodrigues(rotation)[0][:, 2]
    center = translation[:, 0]
    if normal @ center < 0: normal = -normal
    points, _ = cv2.projectPoints(np.asarray([center, center + normal * .12]), np.zeros(3), np.zeros(3), k, None)
    points = np.rint(points[:, 0]).astype(int)
    return tuple(map(tuple, points))


def battery_box_pixels(transform, bounds, base_from_color, intrinsics):
    bounds = np.asarray(bounds, float) / 1000.
    corners_local = np.asarray([[bounds[x, 0], bounds[y, 1], bounds[z, 2]]
                                for x in (0, 1) for y in (0, 1) for z in (0, 1)])
    corners_base = corners_local @ transform[:3, :3].T + transform[:3, 3]
    return [project_base(point, base_from_color, intrinsics) for point in corners_base]


def edges_from_pixels(pixels):
    pairs = []
    for i in range(8):
        for bit in (1, 2, 4):
            j = i ^ bit
            if i < j and pixels[i] is not None and pixels[j] is not None:
                pairs.append((pixels[i], pixels[j]))
    return pairs


def battery_box_edges(transform, bounds, base_from_color, intrinsics):
    return edges_from_pixels(battery_box_pixels(transform, bounds, base_from_color, intrinsics))


def _ordered_quad(points):
    points = np.asarray(points, np.float32)
    center = points.mean(0)
    points = points[np.argsort(np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0]))]
    return np.roll(points, -int(np.argmin(points.sum(1))), axis=0)


def _gray_battery_top_face(rgb, xyz, expected_px, expected_depth_m=None):
    """Find the gray upper battery face and require paired depth on that face."""
    rgb = np.asarray(rgb, np.uint8); xyz = np.asarray(xyz, float)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or xyz.shape != (*rgb.shape[:2], 3):
        return None
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = ((hsv[:, :, 0] >= 85) & (hsv[:, :, 0] <= 115) & (hsv[:, :, 1] >= 12) &
            (hsv[:, :, 1] <= 75) & (hsv[:, :, 2] >= 85) & (hsv[:, :, 2] <= 210)).astype(np.uint8)
    x, y = map(int, expected_px); roi = np.zeros(mask.shape, np.uint8)
    roi[max(0, y-180):min(mask.shape[0], y+180), max(0, x-140):min(mask.shape[1], x+140)] = 1
    mask &= roi
    if expected_depth_m is not None and np.isfinite(expected_depth_m):
        mask &= (np.abs(xyz[:, :, 2] - float(expected_depth_m)) <= .16).astype(np.uint8)
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
    choices = []
    for index in range(1, count):
        area = stats[index, cv2.CC_STAT_AREA]
        if area < 500: continue
        valid = (labels == index) & (xyz[:, :, 2] > .1) & (xyz[:, :, 2] < 2.)
        ratio = valid.sum() / area
        if ratio < .55: continue
        choices.append((np.linalg.norm(centers[index] - [x, y]), -area, index))
    if not choices: return None
    index = min(choices)[2]
    selected = (labels == index) & (xyz[:, :, 2] > .1) & (xyz[:, :, 2] < 2.)
    ys, xs = np.nonzero(selected); points = xyz[ys, xs]
    if len(points) < 500: return None
    rng = np.random.default_rng(19); best = None
    for _ in range(80):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b-a, c-a); length = np.linalg.norm(normal)
        if length < 1e-8: continue
        normal /= length; inliers = np.abs((points-a) @ normal) <= .005
        if best is None or inliers.sum() > best.sum(): best = inliers
    if best is None or best.sum() < 500: return None
    component = np.zeros(mask.shape, np.uint8); component[ys[best], xs[best]] = 255
    component = cv2.morphologyEx(component, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
    contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours: return None
    hull = cv2.convexHull(max(contours, key=cv2.contourArea))
    quad = np.empty((0, 2), np.float32)
    for epsilon in (.04, .05, .06, .07, .08, .09, .10):
        quad = cv2.approxPolyDP(hull, epsilon * cv2.arcLength(hull, True), True)[:, 0, :]
        if len(quad) == 4: break
    return _ordered_quad(quad) if len(quad) == 4 else None


def battery_depth_face(rgb, xyz, expected_px, expected_depth_m=None, projected_pixels=None):
    """Return a bounded depth-plane observation for a partly clipped battery face."""
    rgb = np.asarray(rgb, np.uint8); xyz = np.asarray(xyz, float)
    if (rgb.ndim != 3 or rgb.shape[2] != 3 or xyz.shape != (*rgb.shape[:2], 3)
            or expected_px is None or len(expected_px) != 2):
        return None
    height, width = rgb.shape[:2]
    x, y = np.rint(expected_px).astype(int)
    if not (np.isfinite([x, y]).all() and -width <= x < width * 2 and -height <= y < height * 2):
        return None

    roi = np.zeros((height, width), np.uint8)
    projected = [] if projected_pixels is None else [point for point in projected_pixels
                                                      if point is not None and np.isfinite(point).all()]
    if len(projected) >= 3:
        hull = cv2.convexHull(np.rint(projected).astype(np.int32))
        cv2.fillConvexPoly(roi, hull, 1)
        roi = cv2.dilate(roi, np.ones((31, 31), np.uint8))
    else:
        # Keep the fallback tighter than the projected-cuboid path so the table
        # plane below a boundary-clipped battery cannot dominate RANSAC.
        roi[max(0, y-120):min(height, y+120), max(0, x-130):min(width, x+130)] = 1

    depth = xyz[:, :, 2]
    valid = (roi > 0) & np.isfinite(depth) & (depth > .1) & (depth < 2.)
    if expected_depth_m is not None and np.isfinite(expected_depth_m):
        # The tracked reference supplies a strong 2-D prior. Its absolute depth can
        # drift closer when the taught grasp is reloaded. A surface materially behind
        # the predicted center cannot be the held battery and is rejected tightly.
        delta = depth - float(expected_depth_m)
        valid &= (delta >= -.30) & (delta <= .08)
    ys, xs = np.nonzero(valid)
    if len(xs) < 500:
        return None
    points = xyz[ys, xs]
    if not np.isfinite(points).all():
        return None

    rng = np.random.default_rng(19)
    sample_indices = np.arange(len(points))
    if len(sample_indices) > 14000:
        sample_indices = rng.choice(sample_indices, 14000, replace=False)
    sample = points[sample_indices]
    best = None
    for _ in range(180):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(b-a, c-a); length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        inliers = np.abs((points-a) @ normal) <= .006
        count = int(inliers.sum())
        if best is None or count > best[0]:
            best = count, normal, a, inliers
    if best is None or best[0] < 500:
        return None

    count, normal, anchor, inliers = best
    plane_mask = np.zeros((height, width), np.uint8)
    plane_mask[ys[inliers], xs[inliers]] = 1
    plane_mask = cv2.morphologyEx(plane_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    components, labels, stats, centers = cv2.connectedComponentsWithStats(plane_mask, 8)
    choices = []
    for index in range(1, components):
        area = int(stats[index, cv2.CC_STAT_AREA])
        left, top, box_width, box_height = map(int, stats[index, :4])
        if area < 500 or box_width < 28 or box_height < 28:
            continue
        aspect = box_width / float(box_height)
        fill = area / float(box_width * box_height)
        if not .25 <= aspect <= 4.0 or fill < .12:
            continue
        distance = float(np.linalg.norm(centers[index] - [x, y]))
        if distance > max(90., .45*np.hypot(box_width, box_height)):
            continue
        choices.append((area / (1. + distance / 80.), area, -distance, index, fill))
    if not choices:
        return None
    _, area, neg_distance, index, fill = max(choices)
    component = (labels == index).astype(np.uint8) * 255
    contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    hull = cv2.convexHull(contour)
    if cv2.contourArea(hull) < 450:
        return None
    quad = cv2.boxPoints(cv2.minAreaRect(hull))
    quad[:, 0] = np.clip(quad[:, 0], 0, width-1)
    quad[:, 1] = np.clip(quad[:, 1], 0, height-1)
    quad = _ordered_quad(quad)

    left, top, box_width, box_height = map(int, stats[index, :4])
    right = left + box_width - 1; bottom = top + box_height - 1
    contacts = []
    if top <= 1: contacts.append('top')
    if right >= width-2: contacts.append('right')
    if bottom >= height-2: contacts.append('bottom')
    if left <= 1: contacts.append('left')
    selected_depth = depth[labels == index]
    delta = None if expected_depth_m is None or not np.isfinite(expected_depth_m) else float(np.median(selected_depth)-expected_depth_m)
    if len(projected) >= 3:
        projected_area = cv2.contourArea(cv2.convexHull(np.asarray(projected, np.float32)))
    else:
        projected_area = box_width*box_height
    visible_fraction = float(np.clip(area / max(1., projected_area), 0., 1.))
    confidence = float(np.clip(.35*(count/len(points)) + .35*visible_fraction
                               + .30*(1.-min(1., -neg_distance/90.)), 0., 1.))
    return dict(quad=quad, contour=contour[:, 0, :].astype(float), method='depth_partial_plane',
                visible_fraction=visible_fraction, boundary_contacts=contacts, confidence=confidence,
                plane_normal=np.asarray(normal, float), plane_anchor=np.asarray(anchor, float),
                median_depth_m=float(np.median(selected_depth)), expected_depth_delta_m=delta)


def battery_top_face(rgb, xyz, expected_px, expected_depth_m=None):
    """Keep the calibrated gray-face path, then accept a bounded depth-only partial face."""
    rgb_array = np.asarray(rgb)
    xyz_array = np.asarray(xyz)
    if rgb_array.ndim != 3 or rgb_array.shape[2] != 3 or xyz_array.shape != (*rgb_array.shape[:2], 3):
        return None
    quad = _gray_battery_top_face(rgb, xyz, expected_px, expected_depth_m)
    if quad is not None:
        return quad
    observation = battery_depth_face(rgb, xyz, expected_px, expected_depth_m)
    return None if observation is None else observation['quad']


def visible_face_overlay(quad):
    if quad is None: return [], None
    q = _ordered_quad(quad)
    edges = [(tuple(np.rint(q[i]).astype(int)), tuple(np.rint(q[(i+1)%4]).astype(int))) for i in range(4)]
    cabinet_end = tuple(np.rint((q[0] + q[1]) / 2).astype(int))
    gripper_end = tuple(np.rint((q[2] + q[3]) / 2).astype(int))
    return edges, (gripper_end, cabinet_end)


def _draw_dashed_line(canvas, start, end, color, thickness=2, dash_px=9):
    start = np.asarray(start, float); end = np.asarray(end, float)
    length = float(np.linalg.norm(end-start))
    if not np.isfinite(length) or length < 1:
        return
    direction = (end-start)/length
    for offset in np.arange(0., length, dash_px*2):
        a = start + direction*offset
        b = start + direction*min(length, offset+dash_px)
        cv2.line(canvas, tuple(np.rint(a).astype(int)), tuple(np.rint(b).astype(int)),
                 color, thickness, cv2.LINE_AA)


def battery_face_evidence(rgb, xyz, expected_px, expected_depth_m, projected_pixels=None):
    """Choose legacy or partial evidence once so live and compatibility paths agree."""
    gray = _gray_battery_top_face(rgb, xyz, expected_px, expected_depth_m)
    if gray is not None:
        edges, axis = visible_face_overlay(gray)
        return dict(method='legacy_gray_face', battery_edges=edges, battery_axis=axis,
                    measured_contour=None, inferred_edges=[], observation=None,
                    model_completion_aligned=True, measured_depth_supported=True)
    observation = battery_depth_face(rgb, xyz, expected_px, expected_depth_m,
                                     projected_pixels=projected_pixels)
    if observation is None:
        return dict(method='unavailable', battery_edges=[], battery_axis=None,
                    measured_contour=None, inferred_edges=[], observation=None,
                    model_completion_aligned=False, measured_depth_supported=False)
    _, measured_axis = visible_face_overlay(observation['quad'])
    delta = observation.get('expected_depth_delta_m')
    aligned = delta is not None and np.isfinite(delta) and abs(float(delta)) <= .08
    projected = [] if projected_pixels is None else [point for point in projected_pixels if point is not None]
    inferred = edges_from_pixels(projected_pixels) if aligned and len(projected) == 8 else []
    predicted_axis = None
    if aligned and len(projected) == 8:
        points = np.asarray(projected, float)
        predicted_axis = (tuple(np.rint(points.mean(0)).astype(int)),
                          tuple(np.rint(points[[0, 2, 4, 6]].mean(0)).astype(int)))
    return dict(method='depth_partial_plane', battery_edges=[], battery_axis=predicted_axis or measured_axis,
                measured_contour=observation['contour'], inferred_edges=inferred, observation=observation,
                model_completion_aligned=bool(aligned), measured_depth_supported=True)


def battery_upper_face(rgb, xyz):
    """Detect the user-approved top/right, camera-facing battery plane without a pose prior."""
    rgb = np.asarray(rgb, np.uint8); xyz = np.asarray(xyz, float)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or xyz.shape != (*rgb.shape[:2], 3):
        return None
    height, width = rgb.shape[:2]
    depth = xyz[:, :, 2]
    roi = np.zeros((height, width), bool)
    roi[:max(1, int(round(height*.55))), int(round(width*.46)):] = True
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
    close_kernel = np.ones((max(5, int(round(height*.065))),
                            max(3, int(round(width*.014)))), np.uint8)
    for normal, anchor in planes:
        facing = abs(float(normal[2]))
        if facing < .90:
            continue
        inliers = np.abs((points-anchor) @ normal) <= .008
        mask = np.zeros((height, width), np.uint8)
        mask[ys[inliers], xs[inliers]] = 1
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
        for index in range(1, count):
            area = int(stats[index, cv2.CC_STAT_AREA])
            left, top, box_width, box_height = map(int, stats[index, :4])
            if (area < max(500, int(width*height*.004)) or
                    box_width < max(28, int(width*.10)) or box_height < max(20, int(height*.08))):
                continue
            right = left + box_width - 1
            aspect = box_width / float(box_height)
            if not .40 <= aspect <= 5.0:
                continue
            if top > max(2, int(height*.03)) or right < width-2 or centers[index, 0] < width*.60:
                continue
            component = labels == index
            median_depth = float(np.median(depth[component]))
            score = area*facing/(median_depth+.1)
            candidates.append((score, area, facing, median_depth, component, normal, anchor))
    if not candidates:
        return None
    _, area, facing, median_depth, component, normal, anchor = max(candidates, key=lambda row: row[0])
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
    visible_fraction = float(np.clip(area/rect_area, 0., 1.))
    confidence = float(np.clip(.45*facing + .35*visible_fraction + .20*min(1., area/(width*height*.05)), 0., 1.))
    return dict(method='upper_front_depth_plane', quad=quad, contour=quad.copy(),
                median_depth_m=median_depth, plane_normal=np.asarray(normal, float),
                plane_anchor=np.asarray(anchor, float), boundary_contacts=contacts,
                visible_fraction=visible_fraction, confidence=confidence,
                insertion_axis_available=False, hidden_space_clearance_proven=False)


def battery_upper_face_evidence(rgb, xyz):
    observation = battery_upper_face(rgb, xyz)
    if observation is None:
        return dict(method='unavailable', battery_edges=[], battery_axis=None,
                    measured_contour=None, inferred_edges=[], observation=None,
                    model_completion_aligned=False, measured_depth_supported=False,
                    insertion_axis_available=False, hidden_space_clearance_proven=False)
    return dict(method=observation['method'], battery_edges=[], battery_axis=None,
                measured_contour=observation['quad'], inferred_edges=[], observation=observation,
                model_completion_aligned=False, measured_depth_supported=True,
                insertion_axis_available=False, hidden_space_clearance_proven=False)


def write_depth_map(xyz, path, near_m=.3, far_m=1.2):
    depth = np.asarray(xyz, float)[:, :, 2]
    valid = np.isfinite(depth) & (depth >= near_m) & (depth <= far_m)
    scaled = np.zeros(depth.shape, np.uint8)
    scaled[valid] = np.clip((far_m-depth[valid])/(far_m-near_m)*255, 0, 255).astype(np.uint8)
    colored = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO); colored[~valid] = 0
    cv2.putText(colored, f'NEAR {near_m:.2f}m', (14, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(colored, f'FAR {far_m:.2f}m', (14, 54), cv2.FONT_HERSHEY_SIMPLEX, .65, (255,255,255), 2, cv2.LINE_AA)
    temporary = Path(path).with_name(Path(path).stem+'.tmp'+Path(path).suffix)
    if not cv2.imwrite(str(temporary), colored, [cv2.IMWRITE_JPEG_QUALITY, 92]): raise OSError('D435 深度图写入失败')
    os.replace(temporary, path)


def snap_wireframe_to_top_face(pixels, axis, measured_quad, intrinsics=None, bounds_mm=None, expected_depth_m=None):
    """Solve a 3D cuboid pose from its observed z-min face, then reproject all edges."""
    if measured_quad is None or any(point is None for point in pixels) or intrinsics is None or bounds_mm is None:
        return edges_from_pixels(pixels), axis, False
    bounds = np.asarray(bounds_mm, float) / 1000.; quad = _ordered_quad(measured_quad)
    face = np.asarray([[bounds[0,0],bounds[0,1],bounds[0,2]], [bounds[1,0],bounds[0,1],bounds[0,2]],
                       [bounds[1,0],bounds[1,1],bounds[0,2]], [bounds[0,0],bounds[1,1],bounds[0,2]]], np.float32)
    corners = np.asarray([[bounds[x,0],bounds[y,1],bounds[z,2]] for x in (0,1) for y in (0,1) for z in (0,1)], np.float32)
    center = bounds.mean(0).astype(np.float32); front = center.copy(); front[2] = bounds[0,2]
    k = np.asarray([[intrinsics['fx'],0,intrinsics.get('cx',intrinsics.get('ppx'))],
                    [0,intrinsics['fy'],intrinsics.get('cy',intrinsics.get('ppy'))],[0,0,1]],float)
    predicted = np.asarray(axis, float) if axis is not None else None
    predicted_vector = None if predicted is None else predicted[1] - predicted[0]
    best = None
    for reverse in (False, True):
        base = quad if not reverse else quad[[0,3,2,1]]
        for shift in range(4):
            image = np.roll(base, shift, axis=0)
            ok, rotation, translation = cv2.solvePnP(face, image, k, None, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok: continue
            projected, _ = cv2.projectPoints(np.vstack((corners, center, front)), rotation, translation, k, None)
            projected = projected[:,0,:]
            vector = projected[-1] - projected[-2]; length = np.linalg.norm(vector)
            if not 12 < length < 400 or translation[2,0] <= .05: continue
            score = 0. if predicted_vector is None else float(vector @ predicted_vector / (length*np.linalg.norm(predicted_vector)+1e-9))
            if expected_depth_m is not None: score -= abs(float(translation[2,0])-float(expected_depth_m))*2.
            if best is None or score > best[0]: best = (score, projected)
    if best is None: return edges_from_pixels(pixels), axis, False
    projected = [tuple(np.rint(point).astype(int)) for point in best[1][:-2]]
    projected_axis = tuple(tuple(np.rint(point).astype(int)) for point in best[1][-2:])
    return edges_from_pixels(projected), projected_axis, True


def _atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')))
    os.replace(temporary, path)


class LiveOverlay:
    def __init__(self, capture, root=ROOT, output=None):
        self.capture = capture
        self.root = Path(root)
        self.output = Path(output or self.root / 'web/public/camera-overlay')
        self.output.mkdir(parents=True, exist_ok=True)
        self.model = ColumnModel(side='right')
        self.tcp = np.asarray(json.loads((self.root / 'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'], float)
        self.bounds = np.asarray(json.loads((self.root / 'config/battery-envelope.json').read_text())['bounds_mm'], float)
        self.lock = threading.Lock()
        self.state = dict(status='idle', reason='等待 D435 叠加画面', running=False, updated_at=0.)

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def refresh(self):
        with self.lock:
            if self.state.get('running'):
                return dict(self.state)
            self.state = dict(self.state, status='capturing', reason='正在读取 D435 配对帧', running=True)
        threading.Thread(target=self._run, daemon=True, name='d435-live-overlay').start()
        return self.snapshot()

    def _reference(self):
        preferred = self.root / 'reports/d435-visual-level-active/estimate-1795067389284369934/pose.json'
        candidates = [preferred] if preferred.is_file() else []
        candidates = sorted((self.root / 'reports/d435-visual-level-active').glob('estimate-*/pose.json'),
                            key=lambda path: path.stat().st_mtime, reverse=True)
        if preferred.is_file(): candidates = [preferred] + [path for path in candidates if path != preferred]
        for pose_path in candidates:
            pose = json.loads(pose_path.read_text())
            frame_path = pose_path.with_name('frame.npz')
            if pose.get('quality', {}).get('status') != 'consistent' or not frame_path.is_file():
                continue
            with np.load(frame_path, allow_pickle=False) as frame:
                return (np.asarray(pose['T_base_from_object'], float), np.asarray(frame['joints'], float),
                        float(frame['lift_rad']), str(pose.get('id', '')))
        raise ValueError('没有可用于跟踪当前夹持电池的已确认 D435 位姿')

    @staticmethod
    def _axis_points(transform, bounds):
        center_local = bounds.mean(0) / 1000.
        # The cabinet-facing leading face is object -Z (z-min); object +Z is the gripped end.
        front_local = center_local.copy()
        front_local[2] = bounds[0, 2] / 1000.
        return (transform[:3, :3] @ center_local + transform[:3, 3],
                transform[:3, :3] @ front_local + transform[:3, 3])

    def _run(self):
        try:
            self.capture.handle_action({'op': 'capture'})
            frame_path = self.root / 'web/hardware/runtime/grasp-head/frame.npz'
            with np.load(frame_path, allow_pickle=False) as source:
                frame = {key: source[key].copy() for key in source.files}
            frame['id'] = str(frame['id']); frame['camera_id'] = str(frame['camera_id'])
            frame['calibration_sha256'] = str(frame['calibration_sha256'])
            intrinsics = json.loads(str(frame['intrinsics']))
            evidence = battery_upper_face_evidence(frame['rgb'], frame['xyz'])
            edges = evidence['battery_edges']; battery_axis = evidence['battery_axis']
            measured_contour = evidence['measured_contour']; inferred_edges = evidence['inferred_edges']
            observation = evidence['observation'] or dict(method=evidence['method'])
            depth_aligned = evidence['model_completion_aligned']
            detection = detect(frame['rgb']); cabinet_axis = None; cabinet_reason = detection.get('reason', '')
            if detection.get('status') == 'complete':
                cabinet_axis = cabinet_axis_from_quad(detection['inner_quad_px'], intrinsics)
                if cabinet_axis is not None:
                    cabinet_reason = ''
            result = self.render(frame['rgb'], edges, detection, battery_axis,
                                 cabinet_axis, frame['id'], cabinet_reason=cabinet_reason, reference_id='',
                                 depth_aligned=depth_aligned, xyz=frame['xyz'], measured_contour=measured_contour,
                                 inferred_edges=inferred_edges, battery_observation=observation,
                                 measured_depth_supported=evidence['measured_depth_supported'])
            with self.lock: self.state = dict(result, running=False, updated_at=time.time(), reason=result.get('reason', ''))
        except Exception as error:
            data = dict(sequence=str(time.time_ns()), updated_at=time.time(), stale=False, error=str(error),
                        status='unavailable', battery_axis=dict(status='unavailable'), cabinet_axis=dict(status='unavailable'))
            _atomic_json(self.output / 'd435.json', data)
            with self.lock: self.state = dict(status='unavailable', running=False, updated_at=time.time(), reason=str(error))

    def render(self, rgb, battery_edges, detection, battery_axis, cabinet_axis, sequence,
               *, cabinet_reason='', reference_id='', depth_aligned=False, xyz=None,
               measured_contour=None, inferred_edges=None, battery_observation=None,
               measured_depth_supported=False):
        self.output.mkdir(parents=True, exist_ok=True)
        canvas = cv2.cvtColor(np.asarray(rgb, np.uint8), cv2.COLOR_RGB2BGR)
        observation = battery_observation or {}
        for start, end in battery_edges:
            cv2.line(canvas, start, end, (255, 0, 255), 3, cv2.LINE_AA)
        if measured_contour is not None and len(measured_contour) >= 3:
            cv2.polylines(canvas, [np.rint(measured_contour).astype(np.int32)], True,
                          (255, 0, 255), 4, cv2.LINE_AA)
            if observation.get('method') == 'upper_front_depth_plane':
                points = np.asarray(measured_contour, float)
                label = (max(8, int(points[:, 0].min())-4),
                         min(canvas.shape[0]-10, int(points[:, 1].max())+24))
                cv2.putText(canvas, 'BATTERY FACE', label, cv2.FONT_HERSHEY_SIMPLEX,
                            .58, (255, 0, 255), 2, cv2.LINE_AA)
        for start, end in inferred_edges or []:
            _draw_dashed_line(canvas, start, end, (255, 80, 190), 2)
        if detection.get('outer_contour_px'): cv2.polylines(canvas, [np.asarray(detection['outer_contour_px'], np.int32)], True, (0, 0, 255), 4, cv2.LINE_AA)
        if detection.get('inner_contour_px'): cv2.polylines(canvas, [np.asarray(detection['inner_contour_px'], np.int32)], True, (0, 0, 255), 3, cv2.LINE_AA)
        if battery_axis and all(point is not None for point in battery_axis):
            cv2.arrowedLine(canvas, battery_axis[0], battery_axis[1], (0, 255, 255), 4, cv2.LINE_AA, tipLength=.18)
            cv2.putText(canvas, 'BATTERY FRONT', (battery_axis[0][0] + 8, battery_axis[0][1] - 8), cv2.FONT_HERSHEY_SIMPLEX, .58, (0, 255, 255), 2, cv2.LINE_AA)
        if cabinet_axis and all(point is not None for point in cabinet_axis):
            cv2.arrowedLine(canvas, cabinet_axis[0], cabinet_axis[1], (255, 255, 0), 4, cv2.LINE_AA, tipLength=.18)
            cv2.putText(canvas, 'CABINET IN', (cabinet_axis[0][0] + 8, cabinet_axis[0][1] - 8), cv2.FONT_HERSHEY_SIMPLEX, .58, (255, 255, 0), 2, cv2.LINE_AA)
        temporary = self.output / 'd435.tmp.jpg'
        if not cv2.imwrite(str(temporary), canvas, [cv2.IMWRITE_JPEG_QUALITY, 92]): raise OSError('D435 叠加画面写入失败')
        os.replace(temporary, self.output / 'd435.jpg')
        if xyz is not None: write_depth_map(xyz, self.output / 'd435-depth.jpg')
        battery_visible = bool(battery_edges) or measured_contour is not None
        battery_status = 'visible' if battery_axis else 'unavailable'; cabinet_status = 'visible' if cabinet_axis else 'unavailable'
        reason = '' if cabinet_status == 'visible' else (cabinet_reason or '仓口进入轴未识别')
        metadata = dict(sequence=str(sequence), updated_at=time.time(), stale=False, error='', status='ready',
                        battery_contour=dict(status='visible' if battery_visible else 'unavailable', color='magenta',
                                             method=observation.get('method', 'current depth-supported visible face only'),
                                             visible_fraction=float(observation.get('visible_fraction', 1. if battery_edges else 0.)),
                                             boundary_contacts=list(observation.get('boundary_contacts', [])),
                                             confidence=float(observation.get('confidence', 1. if battery_edges else 0.)),
                                             inferred_completion=bool(inferred_edges),
                                             model_completion_aligned=bool(depth_aligned),
                                             measured_depth_supported=bool(measured_depth_supported or battery_edges),
                                             insertion_axis_available=bool(observation.get('insertion_axis_available', battery_axis is not None)),
                                             hidden_space_clearance_proven=False),
                        cabinet_rim=dict(status='visible' if detection.get('status') == 'complete' else detection.get('status', 'unavailable'), color='red'),
                        battery_axis=dict(status=battery_status, color='yellow',
                                          direction='object -Z / cabinet-facing leading face' if battery_axis else 'unavailable'),
                        cabinet_axis=dict(status=cabinet_status, color='cyan', direction='inside'),
                        reference_pose_id=reference_id, depth_top_face_aligned=bool(depth_aligned), reason=reason)
        _atomic_json(self.output / 'd435.json', metadata)
        return dict(status='ready', sequence=str(sequence), reason=reason, battery_axis=metadata['battery_axis'], cabinet_axis=metadata['cabinet_axis'])
