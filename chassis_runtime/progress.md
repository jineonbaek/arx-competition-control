# ROS chassis migration ledger — 2026-11-20
Task 1: complete — pure core regressions and TTL/profile/maintenance tests passed.
Task 2: complete — typed interfaces built; isolated domain199 fake-driver tests passed; web/session restart regressions passed.
Task 3: complete — Controller proxy, native navigation/gamepad facade, typed bound driver compiled. Observer maintenance heartbeat is asynchronous.
Final review: independent reviewer identified source-role initialization, expired maintenance capability, restart generation replay, source clock skew, output-reader readiness and float64 serialization issues. Regression tests cover fixes.
Ruling: preserve legacy pure in-process scheduler only for non-ROS unit fixtures; production config selects ROS and RosAdapter direct chassis publication always rejects.
Ruling: management operations share one typed /chassis/manage service, while motion uses /chassis/command and /chassis/command_result. Per-source TTL and original deadline remain enforced.
Task 4: complete — live sole output, web-independent native zero stream, scheduler epoch fencing, HTTP integer TTL and domain192→191 context checks passed. See NUC navigation/reports/ros-chassis-20261120/*.json. No nonzero motion sent.
