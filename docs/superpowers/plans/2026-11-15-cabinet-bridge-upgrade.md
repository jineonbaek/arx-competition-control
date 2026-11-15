# Cabinet bridge upgrade, publish and test

User explicitly authorized upgrading, pushing and testing on 2026-11-15, without unnecessary confirmation. This supersedes the earlier no-push/no-service-switch constraint for this upgrade. Physical collision and evidence gates remain binding.

## Scope

- Preserve `/home/arx/LIFT`, ROS2_LIFT_Play and docs_lift2. Preserve current runtime navigation and competition changes.
- Publish the related independent placement work on `codex/automatic-placement`; do not merge or force-push main.
- Apply only the approved driver-binding extension to the newer installed Controller. Never replace it with the older independent Controller wholesale.
- Install the bridge dependencies and a readiness/admission-probe interface, default execution disabled because no physical evidence provider is installed.
- Restart the web observer only when tasks and motion ownership are idle. Do not restart motor drivers or camera owner; preserve their process identities.
- Tests: fake transport regression suites, newer Controller compatibility matrix, online non-motion rejection probe, actual feedback and camera continuity.

## Steps

- [x] Add failing tests for readiness and a fixed admission probe that cannot accept motion targets or enable overrides.
- [x] Implement pure diagnostic helpers and protected GET/POST routes; test using FakeAdapter.
- [x] Build an independent candidate from current runtime source plus narrow patches, preserving all newer features. Test before deployment.
- [x] Back up, compare source hashes, apply checked files, restart only idle web service and verify/roll back on failure.
- [x] Commit explicit related source/docs/tests only and push the feature branch; confirm remote commit identity.
- [x] Record actual results, live execution blockers, test layers and deployed files.

## Execution evidence

Readiness tests first failed on missing module, then passed; HTTP test exposed missing-feedback KeyError, now explicitly rejected. Final reviewer found no consequential issue (8 fake-only readiness tests). Full author suites: 370+441+50=861; newer runtime candidate matrix:96, preserving original runtime preview fixture and adding only new probe tests.

First deployment automatically rolled back on initial trajectory-subscription discovery delay; read-only ROS echo confirmed existing driver idle/ready. Bounded discovery wait fixed the deployment check; second deployment passed without changing driver policy or gates. Backup bridge-web-upgrade-20261115-105434. No actuator command or driver/camera restart. Physical provider remains absent and execution disabled.

Source commit/push and remote identity confirmation recorded in the feature branch history and final report. This plan's earlier no-push history is superseded only by the latest explicit user authorization.
