"""Update one removed PyTorch import, preserving upstream bytes and line endings."""
from pathlib import Path

def main():
    path = Path(__file__).resolve().parent / "upstream/graspnet-baseline/dataset/graspnet_dataset.py"
    source = path.read_bytes()
    old = b"from torch._six import container_abcs"
    new = b"import collections.abc as container_abcs"
    if source.count(old) == 1:
        path.write_bytes(source.replace(old, new))
    elif old not in source and source.count(new) == 1:
        return
    else:
        raise RuntimeError("Unexpected upstream dataset import; compatibility change refused")

if __name__ == "__main__":
    main()
