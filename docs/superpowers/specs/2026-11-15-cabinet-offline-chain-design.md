# Cabinet preview chain, offline only

User scope: inspect and improve placement without a physical test. Preserve the original projects, the dirty placement worktree, and the running robot processes. No control commands, gripper release, driver restart, git push, or automatic motion approval.

Build on the existing Python/OpenCV preview and browser. D435 is the sole primary target; D405 remains auxiliary and may show a clipped opening. The existing measured wrist-to-grip TCP is reused. The D435 camera-to-base transform is nominal CAD and must be shown as unmeasured. A preview must never imply that it can execute.

The current backend requires RGB-D, fits a depth plane, reports point-cloud collision counts despite the user's invalid-depth notice, and returns cached coordinates without checking their age or robot state. Replace only the cabinet preview capture/localization with RGB plus the known rectangular CAD aperture. Keep the existing depth geometry available for older callers, but do not use it for this preview. Reject unsupported distortion, invalid timestamps, mismatched camera identity, clipped/ambiguous red frames, poor reprojection and invalid transforms. Report planar PnP ambiguity and CAD scale assumptions explicitly. PnP pixel agreement does not measure physical positioning accuracy.

Alternatives: repairing depth calibration requires physical evidence outside this task. Retaining a 2D-only detector leaves the coordinate chain unexercised. RGB/CAD PnP is appropriate for offline chain validation, with base coordinates marked nominal and execution disabled.

Every result binds the exposure-time arm/lift/command epoch, calibration bytes, TCP, model and geometry source hashes. A later GET returns a cleared stale result after age, feedback, pose, motion, command epoch or source changes. Frozen results remain visible only as historical images, not current placement coordinates. Browser network failure also clears current coordinates. Snapshot validity is separate from execution readiness.

Add a standalone replay tool taking saved NPZ files. It invokes the same analyzer at the historical exposure time, writes annotated JPEG and JSON, identifies replay mode and actual age, and has no controller/ROS dependency. It never captures a camera or sends commands.

Validation: synthetic pose projection including nonzero supported distortion, missing depth, partial/multiple frames, pose ambiguity, bad metadata and calibration; preview integration with a command-rejecting fake controller; stale/changed-state GET and browser clearing; replay a real saved D435 frame. Physical calibration, carried-object collision acceptance, insertion and release remain unverified and disabled.
