# Loaded arrival recovery

User: correct arrival deviation and continue a bounded insertion test, never release the gripper. No tolerance relaxation, gain/torque change, motion replay or unsupported driver restart.

Evidence: first outside reference completed; measured joints remained up to 1.1043 degrees away and FK TCP error was 6.591 mm. The driver then replaced the issued reference with measured joints on stop. Further measured sag followed. The latter is a distinct recovery defect, not proof that the original load tracking bias is solved.

First deliverable: retain the last issued joint reference only for an explicit manual stop after reference completion, current position mode 5, and a valid reference no more than 2 degrees from measured on every joint. Active trajectory failures, mode transitions, startup and invalid references retain the old measured-position stop behavior. Never use a future goal as the hold reference.

Expose issued reference and commanded gripper target in existing driver status so any future correction uses what was actually sent. Permit an explicit validated startup gripper target for a supported maintenance restart; default startup behavior remains unchanged. No new parameter may command motion on a running instance.

All changes are staged in the independent workspace. Build/test without opening CAN. Live switch requires the battery and right arm to be reliably mechanically supported while the driver briefly loses output; a person watching emergency stop is not mechanical support. Do not switch before that state is confirmed. No gripper-open command at any point. No physical accuracy claim from FK alone.

After supported switch, separately diagnose and validate bounded outside correction before authorizing entry. This patch alone does not authorize or prove insertion.
