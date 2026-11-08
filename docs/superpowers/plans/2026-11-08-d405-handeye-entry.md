# D405 Hand-Eye Preparation Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this small, scoped plan inline. Steps use checkbox syntax for tracking.

**Goal:** Download pinned Seeed reference source to ROG and add a read-only D405 calibration entry to 9983, without collecting samples or executing motion.
**Architecture:** Reuse the existing static server, dark control-console styles, camera owner output and GET feedback. Keep this preparation page isolated from main.js and every command endpoint. No server, driver, dependency or calibration-state changes.
**Tech Stack:** Existing Python standard library, HTML/CSS/JavaScript, Node tests and installed headless Chrome.
**Spec:** User request in this chat on 2026-11-08: focus D405 eye-in-hand; reference Seeed automation; download and add 9983 entry; do not execute; user will supply limits later.

## Scope and choices

The user selected Seeed's automatic sampling flow. This stage prepares a separate static /handeye.html page linked from the D405 card. A separate service or immediate automatic-motion backend adds work outside this stage and would require the limits that have not yet been supplied.
The installation side is not assumed. No board, pose or mechanical limit is invented.
Unrelated dirty server/control/navigation/gamepad work is preserved and not staged.

## Global Constraints

- Work under /home/arx/arx-competition-control; branch main; explicit file whitelist for each local commit.
- No push, service restart, driver startup/homing, CAN write, live sample collection, detector startup or depth enable.
- Preserve all three cameras at 640x480 and all current depth requests unchanged.
- Never run upstream entry scripts or install its arm SDK; retain one camera and one hardware owner.
- All displayed feedback must reject stale data; ready feedback never unlocks collection.

## Review Focus

- Direct page opening and repeated refresh must only issue read requests.
- An installation side must be explicitly selected before displaying arm feedback.
- Frozen/offline camera or stale arm feedback must not retain a current-looking value.
- Start/solve/export actions must stay unavailable even with healthy feedback and side selection.
- Existing D405 depth button and other camera/control UI must remain unchanged.

## Stage 1: Reference preparation

Files: vision/handeye/README.md, vision/handeye/sources.json, this plan.
External files: ignored references/rebot-handeye-52714f61297f.
- [x] Download and verify commit 52714f61297f39a7b9f0d40b0a422e38362b800e via sparse Git checkout.
- [x] Hash key files; parse pose list without executing it; inspect existing OpenCV capabilities.
- [ ] Review diff and commit only preparation documentation/provenance.

## Stage 2: Calibration entry

Files: web/public/index.html, web/public/handeye.html, web/public/css/handeye.css,
web/public/js/handeye.js, web/tests/test_handeye_page.py.
- [ ] Write a headless-browser fixture that renders healthy and stale feedback, verifies the main-page link, disabled actions and GET-only requests; observe failure before implementation.
- [ ] Add a D405 card link and independent preparation page using existing styles and freshness helpers.
- [ ] Reuse existing D405 JPEG output and read arm joints; keep all execution absent.
- [ ] Run relevant camera/UI tests and live GET-only smoke checks; verify process start times and depth state unchanged.
- [ ] Review the scoped diff and create a local commit without teammate changes.

## Status

Prepared from explicit user scope; no further design approval required for this non-executing entry.
