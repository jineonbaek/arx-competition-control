"""Standalone cabinet placement under a T2 task lease, independent of navigation."""
from t2_test_task import T2TestTask


class T2CurrentPositionTask(T2TestTask):
    stages = (('place', '从当前位置开始放置'),)

    def __init__(self, controller, root, *, modules):
        super().__init__(controller, root, modules=modules)

    def snapshot(self, client=None):
        view = super().snapshot(client)
        view['target'] = '当前位置 → 预观测点 → D405 水平修正 → 入仓、松爪并返回'
        return view
