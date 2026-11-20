# Do not use this candidate for loaded restart

2026-11-16: user confirmed independent support, then candidate replaced right-arm process 5160 with process 1186577. Original binary and original engineering directories were not overwritten.

Pre-restart measured J6: -1.5772104263 rad; gripper: -0.8289461136. Last retained gripper command 0.0 confirmed in running-driver stop log and web command history.

Candidate started with resume_clamp=true and resume_clamp_target=0.0. Its status reported ready=true, reference J6=-0.0230789185 rad, gripper_target_raw=0.0. Subsequent measured J6=-0.0268936157, gripper=+0.9201192856. D405 image showed an open gripper and battery on the table. No subsequent approach or fine trajectory was sent.

Therefore retaining the numeric target DID NOT preserve physical grip or wrist pose across restart. The change may involve SDK initialization, coordinate/reference changes or loss of holding during restart; current evidence does not isolate the cause. Manufacturer InterfacesThread implementation is supplied as a binary, only its header was found in the repository.

User subsequently confirmed emergency stop, independent support, and clear personnel area. Candidate received SIGINT; process absence and no can3 lock owner verified. Left and body services were not restarted.

Keep the driver candidate out of production. Do not resume loaded startup from this artifact. First investigate vendor startup behavior with an unloaded, independently supported test setup and an explicit bounded test plan. Existing fine and coarse bridge outputs remain offline proposals, not executable authorization.
