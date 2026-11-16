"""Compute a recorded-frame preview in a separate interpreter; no robot IO."""
import contextlib
import json
from pathlib import Path
import sys
import threading


def main():
    settings = json.load(sys.stdin)
    paths = [settings['candidate_dir'], *settings['support_dirs'], settings['workspace'],
             str(Path(settings['workspace'])/'web/src')]
    sys.path[:0] = paths
    with contextlib.redirect_stdout(sys.stderr):
        from recorded_t2_planner import RecordedT2Planner
        planner = RecordedT2Planner(frame=settings['frame'], pose=settings['pose'],
            workspace=settings['workspace'], control_config=settings['control_config'],
            mode=settings.get('cabinet_mode', 'demo'), d405=settings.get('d405'))
        result = planner(dict(stage='place', cancelled=threading.Event()))
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
