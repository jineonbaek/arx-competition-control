#!/usr/bin/env python3
"""Install the competition CAD delivery and the viewer's pinned JS libraries."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PACKAGE = ROOT / "references/lift2s-20261105/lift2"


def install(path, data):
    if path.exists() and path.read_bytes() == data:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as handle:
        pending = Path(handle.name)
        try:
            handle.write(data)
            handle.flush()
        except Exception:
            pending.unlink(missing_ok=True)
            raise
    try:
        os.replace(pending, path)
    finally:
        pending.unlink(missing_ok=True)


def prepare(manifest, source, public):
    model = manifest["model"]
    package = public / model["public_path"]
    files = {}
    for name, expected in model["sha256"].items():
        local = model.get("local_assets", {}).get(name)
        data = (ROOT / local if local else source / name).read_bytes()
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError(f"Unexpected DIY delivery asset: {name}")
        files[package / name] = data

    rendering = manifest["rendering"]
    for item in rendering["files"]:
        target = public / "vendor/lift2-preview" / item["target"]
        if target.exists() and hashlib.sha256(target.read_bytes()).hexdigest() == item["installed_sha256"]:
            continue
        url = f"https://raw.githubusercontent.com/{rendering['repository']}/{rendering['ref']}/{item['source']}"
        with urllib.request.urlopen(url, timeout=45) as response:
            data = response.read()
        if hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest() != item["git_blob"]:
            raise ValueError(f"Unexpected rendering dependency: {url}")
        if item["target"] in ("OrbitControls.js", "STLLoader.js"):
            data = data.replace(b"from 'three'", b"from './three.module.js'")
        if hashlib.sha256(data).hexdigest() != item["installed_sha256"]:
            raise ValueError(f"Unexpected prepared rendering dependency: {url}")
        files[target] = data
    for path, data in files.items():
        install(path, data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", type=Path, default=DEFAULT_PACKAGE)
    parser.add_argument("--output", type=Path, default=ROOT / "web/public")
    parser.add_argument("--manifest", type=Path, default=ROOT / "web/models-lift2.json")
    args = parser.parse_args()
    prepare(json.loads(args.manifest.read_text()), args.package, args.output)
