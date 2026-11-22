"""Persist an idle-only choice between the existing T2 placement routes."""
from contextlib import ExitStack
from pathlib import Path
import threading

from agent.storage import atomic_json
from chassis_placement import ChassisReleaseModule
from navigation_placement import NavigationPlacement, CurrentPositionPlacement
from placement_policy import CHASSIS_FORWARD_SECONDS, VISUAL_SEATED, LIDAR_SPECIAL_POINT
from t2_timed_placement import TimedT2Placement


OPTIONS = (
    dict(value=LIDAR_SPECIAL_POINT, label='底盘入仓',
         description=f'直接走兜底：预观察点 → 特殊点 → 底盘前进 {CHASSIS_FORWARD_SECONDS:g} 秒 → 松爪退出；跳过 T2 视觉'),
    dict(value=VISUAL_SEATED, label='视觉入仓',
         description='保留旧版：预观察点 → D405 对位 → 机械臂推入 → 坐底确认 → 松爪返回'),
)


class PlacementModeSwitch:
    def __init__(self, controller, cabinet, placement, navigation, root, competition, single):
        self.controller, self.cabinet, self.placement = controller, cabinet, placement
        self.navigation, self.root = navigation, Path(root)
        self.tasks = (competition, single)
        self.lock = threading.RLock()
        self._install(placement.placement_mode, self._build(placement.placement_mode))

    def _build(self, mode):
        whole = NavigationPlacement(self.controller, self.cabinet, self.placement,
                                    self.navigation, self.root, lidar_locked=True)
        if mode == LIDAR_SPECIAL_POINT:
            single = NavigationPlacement(self.controller, self.cabinet, self.placement,
                                         self.navigation, self.root, lidar_locked=True)
            return ((whole, ChassisReleaseModule(whole)), (single, ChassisReleaseModule(single)))
        single = CurrentPositionPlacement(self.controller, self.placement, self.root)
        return tuple((TimedT2Placement(module, self.navigation, self.root), None)
                     for module in (whole, single))

    def _install(self, mode, modules):
        self.placement.placement_mode = mode
        for task, (place, release) in zip(self.tasks, modules):
            task.modules['place'] = place
            task.modules.pop('release_after_chassis', None)
            if release is not None:
                task.modules['release_after_chassis'] = release

    def _busy(self):
        robot = self.controller.snapshot()
        if any(robot.get(k) for k in ('motion_reserved', 'competition_reserved', 'chassis_active')):
            return True
        if self.placement.thread and self.placement.thread.is_alive():
            return True
        return any(task.active and (task.active['worker'].is_alive()
                   or task.active['competition_token'] is not None
                   or task.active['status'] in ('running', 'recovering', 'stopping')) for task in self.tasks)

    def snapshot(self):
        with self.lock:
            return dict(selected=self.placement.placement_mode, options=[dict(o) for o in OPTIONS],
                        can_change=not self._busy())

    def select(self, mode):
        if mode not in (LIDAR_SPECIAL_POINT, VISUAL_SEATED):
            raise ValueError('请选择底盘入仓或视觉入仓')
        with self.lock, ExitStack() as locks:
            for task in self.tasks:
                locks.enter_context(task.lock)
            locks.enter_context(self.placement.lock)
            locks.enter_context(self.controller.lock)
            if self._busy():
                raise ValueError('任务仍在运行或结束处理中，本轮入仓模式保持不变')
            if mode == self.placement.placement_mode:
                return self.snapshot()
            modules = self._build(mode)
            # Write first: a failed save must not silently change the live route.
            atomic_json(self.root/'config/t2-placement.json', dict(version=1, mode=mode))
            self._install(mode, modules)
            # Old receipts remain on disk; continuing them under a new route is invalid.
            for task in self.tasks:
                task.active = None
            return self.snapshot()

    def action(self, task, client, data):
        with self.lock:
            if isinstance(data, dict) and data.get('action') == 'set_placement_mode':
                if (task is not self.tasks[0] or not isinstance(client, str) or not client
                        or set(data) != {'action', 'mode', 'request_id'}
                        or not isinstance(data['request_id'], str) or not 1 <= len(data['request_id']) <= 128):
                    raise ValueError('入仓模式切换请求无效')
                self.select(data['mode'])
                return task.snapshot(client)
            return task.action(client, data)
