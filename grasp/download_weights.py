"""Download the pinned redistributed checkpoint; no model code is executed."""
import hashlib
import json
from pathlib import Path
import os
import requests

ROOT = Path(__file__).resolve().parent
REVISION = "7856e4e550052131b6ac5c0adb6564b04433ad9b"
SHA256 = "60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868"
PATH = f"dgrachev/a2_pretrained/resolve/{REVISION}/checkpoint-rs.tar"
URLS = [f"https://hf-mirror.com/{PATH}", f"https://huggingface.co/{PATH}"]


def main():
    output = ROOT / "models/checkpoint-rs.tar"
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        if hashlib.sha256(output.read_bytes()).hexdigest() != SHA256:
            raise RuntimeError("Existing checkpoint hash differs; it was not overwritten")
        print("Existing checkpoint verified:", SHA256)
        return
    failures = []
    temporary = output.with_suffix(".part")
    for url in URLS:
        try:
            digest = hashlib.sha256()
            with requests.get(url, stream=True, timeout=30) as response:
                response.raise_for_status()
                with temporary.open("wb") as handle:
                    for chunk in response.iter_content(1024*1024):
                        handle.write(chunk)
                        digest.update(chunk)
            if digest.hexdigest() != SHA256:
                raise RuntimeError("Checkpoint checksum mismatch")
            os.replace(temporary, output)
            print("Downloaded checkpoint:", url, output.stat().st_size, SHA256)
            return
        except (OSError, requests.RequestException, RuntimeError) as exc:
            failures.append(f"{url}: {exc}")
            temporary.unlink(missing_ok=True)
    raise RuntimeError("\n".join(failures))


if __name__ == "__main__":
    main()
