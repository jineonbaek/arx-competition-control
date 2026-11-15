# Qwen offline visual decision comparison

Only frozen saved images and cloud visual requests are used. No ROS, SDK, live camera, service restart, or production configuration changes.

- Experiment: enable_thinking=false, max_output_tokens=512, structured result with confidence and one short reason. Confidence is recorded, never used as an acceptance gate.
- Baseline: 8 historical records, 16 logical judgments. New requests are sequential and reuse original candidate boxes and original masks independently. This is not a simultaneous A/B test and changes two factors together.
- Selection mean: historical 20.4815 s, new 2.039 s. Review mean: historical 32.737 s, new 2.1985 s. Total historical 425.748 s, new 33.9 s.
- Manual full-visible-target selection assessment: old 6 correct / 1 partial / 1 wrong; new 5 correct / 1 partial / 1 wrong / 1 uncertain. Original overlapping boxes may refer to the same object.
- Manual mask assessment: one image cannot be confidently labeled and is excluded. Old 4/7 consistent, new 5/7 consistent. Both still wrongly reject the real target mask in image 5 and accept the incomplete mask in image 6.
- Manual labels are visual judgments, not blind, pixel-annotated or competition-authoritative ground truth. Do not interpret the tiny sample as a general accuracy benchmark. Image 7 is the actual saved mask from the 114 s review, not an earlier failed retry mask.

Open report.html for embedded images, judgments and timing tables. comparison.csv is UTF-8 BOM for Excel. results.json preserves raw old/new responses. evaluate.py reruns cloud calls and incurs API usage; it only writes experiment outputs. Existing credentials are read from their existing private environment file and are not copied into artifacts.

Production hashes matched at experiment completion. Checkpoint refs: refs/checkpoints/qwen-offline-20261115-225448 in both source repositories; production tracked dirty work was preserved by a stash-create checkpoint without modifying the active worktree. Untracked existing work was left untouched.
