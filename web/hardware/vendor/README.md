# Lift trajectory dependency

Ruckig Community v0.15.3, MIT licensed, is vendored from
https://github.com/pantor/ruckig at commit
37b6e7a4c60f5befd2506741d5f7d0ae7eefb3db.

The unmodified include/, src/, cmake/, CMakeLists.txt, package.xml, README.md
and LICENSE are retained. SOURCE.json records SHA-256 hashes of copied files.
Our build compiles only the eleven local state-to-state solver sources into a
static library. Cloud support, examples, Python bindings, upstream tests and
network access are not enabled. No global package installation is required.

A fresh project clone includes these sources. To update them, verify the
upstream commit and license, regenerate SOURCE.json, and rerun the project's
motion constraints, retarget, stop and timing tests before any physical release.
