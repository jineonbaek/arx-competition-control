#!/usr/bin/env python3
"""Download pinned, display-only LIFT2 assets. Never imports a robot SDK."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import urllib.request
import xml.etree.ElementTree as ET


def check_blob(data, expected):
    actual = hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()
    if actual != expected:
        raise ValueError("Upstream Git blob mismatch: " + actual)


def fetch(repository, ref, name, blob):
    url = f"https://raw.githubusercontent.com/{repository}/{ref}/{name}"
    with urllib.request.urlopen(url, timeout=45) as response:
        data = response.read()
    check_blob(data, blob)
    return data


def read_member(archive, name, extractor):
    cmd = [extractor, "x", "-so", str(archive), name] if Path(extractor).stem in ("7z", "7zz") else [extractor, "-xOf", str(archive), name]
    return subprocess.run(cmd, check=True, capture_output=True, timeout=45).stdout


def stl_bounds(data):
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + count * 50:
        raise ValueError("Expected binary STL")
    lo, hi = [float("inf")] * 3, [-float("inf")] * 3
    for index in range(count):
        values = struct.unpack_from("<9f", data, 84 + index * 50 + 12)
        for vertex in range(3):
            for axis in range(3):
                v = values[vertex * 3 + axis]
                lo[axis], hi[axis] = min(lo[axis], v), max(hi[axis], v)
    return {"min": lo, "max": hi, "size": [hi[i] - lo[i] for i in range(3)]}


def prepare(manifest, output):
    extractor = shutil.which("7z") or shutil.which("7zz") or shutil.which("bsdtar")
    if extractor is None and os.name == "nt":
        extractor = shutil.which("tar")
    if extractor is None:
        raise RuntimeError("Install p7zip-full/7zip first; this script will not install system packages.")
    model = manifest["model"]
    vendor = output / "vendor/lift2-preview"
    destination = output / "models/lift2"
    bounds = {}
    with tempfile.TemporaryDirectory(prefix="lift2-inspect-") as temp:
        archive = Path(temp) / "lift2.7z"
        archive.write_bytes(fetch(model["repository"], model["ref"], model["archive"], model["git_blob"]))
        source = read_member(archive, "lift2/urdf/lift2.urdf", extractor)
        robot = ET.fromstring(source)
        if robot.get("name") != "lift2" or len(robot.findall("link")) != 21:
            raise ValueError("Unexpected official LIFT2 assembly")
        target = destination / "urdf/lift2.urdf"
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source)
        names = {x.get("filename") for x in robot.findall(".//mesh")}
        for name in sorted(names):
            relative = name.removeprefix("package://lift2/")
            if not relative.startswith("meshes/") or "/" in relative[7:] or not relative.endswith(".STL"):
                raise ValueError("Unexpected mesh reference: " + name)
            data = read_member(archive, "lift2/" + relative, extractor)
            bounds[relative] = stl_bounds(data)
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
    source = manifest["rendering"]
    vendor.mkdir(parents=True, exist_ok=True)
    for item in source["files"]:
        data = fetch(source["repository"], source["ref"], item["source"], item["git_blob"])
        if item["target"] in ("OrbitControls.js", "STLLoader.js"):
            data = data.replace(b"from 'three'", b"from './three.module.js'")
        (vendor / item["target"]).write_bytes(data)
    # Record exact installed bytes after local import-path rewriting.
    hashes = {p.relative_to(output).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for base in (destination, vendor) for p in sorted(base.rglob("*")) if p.is_file()}
    record = {"source": manifest, "sha256": hashes, "mesh_bounds_m": bounds}
    (destination / "asset-integrity.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({"meshes": len(bounds), "installed_files": len(hashes), "mesh_bounds_m": bounds}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=Path(__file__).resolve().parents[1] / "web/models-lift2.json")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "web/public")
    args = parser.parse_args()
    prepare(json.loads(args.manifest.read_text()), args.output)
