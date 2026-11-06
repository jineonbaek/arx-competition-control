"""Read-only snapshots from the existing console camera owner."""
import email.utils
import math
import re
import time
from urllib.request import Request

from .contracts import AgentError, strict_json
from .http import open_request

CAMERAS = ('d435', 'd405', 'astra')


def jpeg_size(body):
    if not body.startswith(b'\xff\xd8') or not body.endswith(b'\xff\xd9'):
        raise ValueError('Invalid JPEG markers')
    offset = 2
    while offset + 4 <= len(body):
        if body[offset] != 255:
            raise ValueError('Invalid JPEG segment')
        marker = body[offset + 1]
        if marker == 255:
            offset += 1
            continue
        length = int.from_bytes(body[offset + 2:offset + 4], 'big')
        if length < 2 or offset + 2 + length > len(body):
            raise ValueError('Truncated JPEG segment')
        if marker in (0xc0, 0xc1, 0xc2):
            height = int.from_bytes(body[offset + 5:offset + 7], 'big')
            width = int.from_bytes(body[offset + 7:offset + 9], 'big')
            if length < 8 or not 0 < width * height <= 16_000_000:
                raise ValueError('Invalid image dimensions')
            return [width, height]
        offset += 2 + length
    raise ValueError('JPEG dimensions missing')


class ConsoleObservations:
    def __init__(self, config, store):
        self.config, self.store = config, store

    def _get(self, route, limit):
        with open_request(Request(self.config.console_url + route, headers={'Accept': '*/*'}), self.config.service_timeout_s) as response:
            body = response.read(limit + 1)
            date = response.headers.get('Date')
        if len(body) > limit:
            raise ValueError('Oversized camera reply')
        # Use the console clock for its own capture timestamps when reading remotely.
        now = email.utils.parsedate_to_datetime(date).timestamp() if date else time.time()
        return body, now

    def robot_state(self):
        try:
            raw, _ = self._get('/api/state', 2_000_000)
            state = strict_json(raw)
            if not isinstance(state, dict) or any(not isinstance(state.get(k), dict) for k in ('arms', 'body', 'ros')):
                raise ValueError('Invalid robot state')
            arm_fields = ('joints', 'gripper', 'tcp', 'timestamp', 'fresh', 'age_s')
            body_fields = ('height', 'wheels', 'waist', 'head_yaw', 'head_pitch', 'timestamp', 'fresh', 'age_s')
            ros_fields = ('connected', 'domain_id', 'error', 'feedback_sources')
            snapshot = {'arms': {side: {key: value for key, value in state['arms'].get(side, {}).items() if key in arm_fields}
                                  for side in ('left', 'right')},
                        'body': {key: value for key, value in state['body'].items() if key in body_fields},
                        'ros': {key: value for key, value in state['ros'].items() if key in ros_fields},
                        'chassis_active': state.get('chassis_active'), 'execution_enabled': state.get('execution_enabled')}
            return {'available': True, 'retrieved_at': time.time(), 'source_time': state.get('time'),
                    'snapshot': snapshot, 'read_only': True,
                    'units_note': 'Controller feedback in SDK conventions; body.height is not calibrated metres. Freshness flags belong to each source.'}
        except (OSError, ValueError, KeyError, TypeError, AttributeError, OverflowError):
            return {'available': False, 'error': 'robot_state_unavailable', 'read_only': True}

    def capture(self, cameras, view='raw'):
        if not cameras or len(cameras) > 3 or len(set(cameras)) != len(cameras) or any(c not in CAMERAS for c in cameras) or view not in ('raw', 'annotated'):
            raise AgentError('invalid_arguments', 'Invalid cameras or view')
        frames, images, errors = {}, {}, {}
        for camera in cameras:
            try:
                suffix = '-battery' if view == 'annotated' else ''
                raw, now = self._get('/camera-live/' + camera + suffix + '.json', 500_000)
                meta = strict_json(raw)
                stamp = meta['captured_at']
                if meta.get('state') != 'online' or type(stamp) not in (int, float) or not math.isfinite(stamp) or not -1.5 <= now - stamp <= self.config.max_frame_age_s:
                    raise ValueError('Camera offline or stale')
                name = (meta.get('raw_image_file') or meta.get('image_file')) if view == 'raw' else meta.get('image_file')
                if not isinstance(name, str) or not re.fullmatch(camera + r'-[vrb]-[a-f0-9]+-[0-9]+\.jpg', name):
                    raise ValueError('Invalid source image name')
                sequence = meta.get('sequence')
                if type(sequence) is not int or int(name.rsplit('-', 1)[1][:-4]) != sequence:
                    raise ValueError('Image and metadata sequence differ')
                body, after = self._get('/camera-live/' + name, self.config.max_image_bytes)
                if after - stamp > self.config.max_frame_age_s:
                    raise ValueError('Frame became stale during download')
                frames[camera] = {'image_size_px': jpeg_size(body), 'source_image_file': name,
                                  'source_sequence': sequence, 'captured_at': stamp,
                                  'view': view, 'metadata': meta, 'geometry_ready': False,
                                  'raw_depth_available': False, 'robot_extrinsics_available': False}
                images[camera] = body
            except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
                errors[camera] = {'code': 'camera_unavailable', 'error_type': type(error).__name__}
        if not frames:
            raise AgentError('observation_unavailable', 'No fresh validated camera frame was available')
        current = self.robot_state()
        robot_state = current.get('snapshot', {})
        if not current['available']:
            errors['robot_state'] = {'code': 'state_unavailable'}
        return self.store.save_observation({'source': 'existing_console_http_get', 'cameras': frames,
                                            'robot_state': robot_state, 'errors': errors,
                                            'cross_camera_synchronized': False,
                                            'robot_state_synchronized': False}, images)
