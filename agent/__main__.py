"""Run from the repository root: python3 -m agent --help."""
import argparse
import json
from pathlib import Path

from .config import Config, load_private_env
from .contracts import AgentError, strict_json
from .model import QwenClient
from .observations import ConsoleObservations
from .runtime import AgentRuntime
from .storage import RunStore
from .tools import ToolRegistry


def output(value):
    print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))


def main(argv=None):
    parser = argparse.ArgumentParser(description='Competition task Agent; physical backends disabled by default')
    parser.add_argument('--config', help='JSON configuration, no secrets')
    parser.add_argument('--env-file', help='Explicit private literal environment file, Linux permissions 600')
    parser.add_argument('--profile', default='cloud', choices=['cloud', 'local'])
    sub = parser.add_subparsers(dest='command', required=True)
    for command in ('doctor', 'tools', 'observe', 'run', 'smoke'):
        p = sub.add_parser(command)
        p.add_argument('--runs-root', default='agent/runs')
        if command in ('observe', 'smoke'):
            p.add_argument('--cameras', nargs='+', choices=['d435', 'd405', 'astra'], default=['d435', 'd405', 'astra'])
        if command == 'observe':
            p.add_argument('--view', default='raw', choices=['raw', 'annotated'])
        if command == 'run':
            p.add_argument('--instruction', required=True)
    for command in ('resume', 'status', 'cancel', 'reconcile'):
        p = sub.add_parser(command)
        p.add_argument('run_dir')
    args = parser.parse_args(argv)
    store = None
    try:
        if args.env_file:
            load_private_env(args.env_file)
        config = Config.load(args.config)
        if args.command in ('status', 'cancel'):
            path = Path(args.run_dir)
            state = strict_json((path / 'state.json').read_text(encoding='utf-8'))
            if args.command == 'cancel':
                (path / 'cancel.flag').touch(mode=0o600)
                output({'cancel_requested': True, 'hardware_stop': False, 'pending': state['pending']})
            else:
                output({k: state[k] for k in ('run_id', 'mission', 'pending', 'outcome')})
            return 0
        if args.command == 'doctor':
            profile = config.profile(args.profile, require_credentials=False)
            from .backends import BackendRouter
            from .mission import OPERATIONS
            backends = BackendRouter(config.services, config.service_timeout_s)
            output({'profile': args.profile, 'model': profile['model'], 'base_url': profile['base_url'],
                    'credentials_present': bool(profile['api_key']) if profile.get('api_key_env') else None,
                    'motion_enabled': config.motion_enabled, 'console_url': config.console_url,
                    'configured_backends': {op: backends.available(op) for op in OPERATIONS},
                    'note': 'Configuration inspection only; no API, camera startup or hardware command'})
            return 0
        store = RunStore.open(args.run_dir) if args.command in ('resume', 'reconcile') else RunStore.create(args.runs_root)
        tools = ToolRegistry(config, store)
        if args.command == 'tools':
            output({'available_tools': tools.definitions(available_only=True), 'capabilities': tools.capabilities()})
        elif args.command == 'observe':
            output(ConsoleObservations(config, store).capture(args.cameras, args.view))
        elif args.command == 'reconcile':
            import uuid
            result = tools.execute('get_execution_status', {}, 'operator_' + uuid.uuid4().hex)
            output(result)
            return 0 if result['ok'] else 2
        elif args.command == 'smoke':
            # Only these two read-only tools are exposed for the connectivity probe.
            model = QwenClient(config, args.profile)
            names = {'get_system_status', 'observe_scene', 'read_observation'}
            real_definitions = tools.definitions
            tools.definitions = lambda **kwargs: [d for d in real_definitions(**kwargs) if d['function']['name'] in names]
            instruction = '只读检查：先调用get_system_status，再调用observe_scene读取相机' + ','.join(args.cameras) + '，根据返回的真实图片用中文简述场景。没有清晰电池就明确说没有；不要建立比赛任务或执行动作。'
            result = AgentRuntime(config, store, tools, model).run(instruction)
            output(result)
            observed = any(c['result']['ok'] and c['result']['tool'] == 'observe_scene' for c in store.state['calls'].values())
            return 0 if result['reason'] == 'model_finished' and observed else 2
        else:
            result = AgentRuntime(config, store, tools, profile=args.profile).run(args.instruction if args.command == 'run' else None)
            output(result)
            return 0 if result['reason'] in ('mission_completed', 'model_finished') else 2
        return 0
    except AgentError as error:
        output({'ok': False, 'error': {'code': error.code, 'message': error.message}})
        return 2
    except (OSError, ValueError, RuntimeError, KeyError):
        output({'ok': False, 'error': {'code': 'configuration_or_run_error', 'message': 'Inspect configuration, run ownership and file availability; no sensitive details printed'}})
        return 2
    finally:
        if store is not None:
            store.close()


if __name__ == '__main__':
    raise SystemExit(main())
