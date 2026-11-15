"""Read-only T1/T2 preflight. No registration, route publication or commands."""
import argparse
import json
from pathlib import Path

from mission_route import MissionRoute
from route_readiness import route_readiness
from task_client import TaskClient


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--file', type=Path, help='Check a saved route definition only')
    source.add_argument('--url', help='Local navigation API (default http://127.0.0.1:9985)')
    args = parser.parse_args(argv)
    try:
        if args.file:
            definition = json.loads(args.file.read_text())
            route = MissionRoute(definition['map_id'])
            route.load(definition)
            result = dict(source='saved_definition', route=route_readiness(
                route.definition(), state='draft', running=False, index=0))
        else:
            state = TaskClient(args.url or 'http://127.0.0.1:9985').state()
            result = dict(source='navigation_state', route=state.get('route'),
                          worker_ready=state.get('worker_ready', False))
        result['scope'] = 'route_configuration_only_not_motion_authorization'
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0 if (result['route'] or {}).get('competition_startable_draft') else 2
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(json.dumps(dict(error=str(exc)), ensure_ascii=False))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
