"""Download pinned FoundationPose weight files through the mainland HF mirror."""
import hashlib
import json
from pathlib import Path
import urllib.request

root=Path(__file__).resolve().parent
manifest=json.loads((root/"weight-sources.json").read_text())
for item in manifest["files"]:
    path=root/"weights"/item["path"]
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest()==item["sha256"]:
        continue
    part=path.with_suffix(path.suffix+".part")
    with urllib.request.urlopen(item["url"],timeout=120) as response,part.open("wb") as out:
        while chunk:=response.read(1024*1024):out.write(chunk)
    if part.stat().st_size!=item["bytes"] or hashlib.sha256(part.read_bytes()).hexdigest()!=item["sha256"]:
        raise ValueError("Checkpoint hash mismatch: "+item["path"])
    part.replace(path)
    print("verified",item["path"])
