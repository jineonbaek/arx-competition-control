"""Read-only competition route diagnostics; never starts or edits a mission."""
import copy


def route_readiness(definition, *, state, running, index):
    tasks = [dict(index=i, route_point_id=p['id'], point_id=p['point_id'],
                  pose=copy.deepcopy(p['pose']))
             for i, p in enumerate(definition['points']) if p['type'] == 'task']
    valid = [p['point_id'] for p in tasks] == ['point_1', 'point_2']
    issues = []
    if not valid:
        issues.append('task_sequence_must_be_point_1_then_point_2')
    if state != 'draft' or running:
        issues.append('explicit_draft_reload_required')
    return dict(route_id=definition['route_id'], map_id=definition['map_id'],
                state=state, running=bool(running), index=index,
                point_count=len(definition['points']), task_points=tasks,
                competition_config_valid=valid,
                competition_startable_draft=not issues, issues=issues)
