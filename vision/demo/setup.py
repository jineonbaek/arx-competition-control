"""Install pinned vision model snapshots; weights and environment remain ignored."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--endpoint", default="https://huggingface.co",
                        choices=["https://huggingface.co", "https://hf-mirror.com"])
    args = parser.parse_args()
    sources = json.loads((ROOT / "sources.json").read_text())
    python = ROOT / ".venv/bin/python"
    if not python.exists():
        subprocess.run([sources["base_python"], "-m", "venv", "--system-site-packages",
                        str(ROOT / ".venv")], check=True)
    if Path(sys.prefix) != ROOT / ".venv":
        subprocess.run([str(python), str(Path(__file__).resolve()), "--endpoint", args.endpoint], check=True)
        return
    os.environ["HF_HUB_DISABLE_XET"] = "1"
    os.environ["HF_HUB_DOWNLOAD_TIMEOUT"] = "30"
    os.environ["HF_HOME"] = str(ROOT / ".cache")
    from huggingface_hub import snapshot_download
    for name, spec in sources["models"].items():
        print("Downloading", name, spec["revision"], args.endpoint, flush=True)
        snapshot_download(spec["repo"], revision=spec["revision"], endpoint=args.endpoint,
                          local_dir=ROOT / "models" / name, max_workers=3,
                          allow_patterns=["*.json", "*.txt", "model.safetensors"])
    subprocess.run([str(python), str(ROOT / "smoke.py")], check=True)

if __name__ == "__main__":
    main()
