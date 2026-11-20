"""Publish complete competition-map snapshots and pin readers to one version."""
from pathlib import Path
from contextlib import contextmanager
import fcntl,hashlib,os,shutil,sys,uuid

@contextmanager
def locked(root):
    root=Path(root);(root/'reports').mkdir(parents=True,exist_ok=True)
    with (root/'reports/competition-map.lock').open('a') as f:
        fcntl.flock(f,fcntl.LOCK_EX);yield

def validate(folder):
    folder=Path(folder)
    if (folder/'INCOMPLETE').exists():raise ValueError('Map is incomplete')
    expected={}
    for line in (folder/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split(None,1);name=name.strip()
        if Path(name).name!=name:raise ValueError('Invalid map manifest')
        expected[name]=digest
    for name in ['room.yaml','room.pgm','room.dual-origin.npz']:
        if name not in expected:raise ValueError('Missing map component: '+name)
    for name,digest in expected.items():
        p=folder/name
        if not p.is_file() or not p.stat().st_size or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:
            raise ValueError('Map checksum mismatch: '+name)

def publish(root,source):
    root=Path(root).resolve();source=Path(source).resolve();validate(source)
    with locked(root):
        versions=root/'reports/competition-map-versions';versions.mkdir(exist_ok=True)
        version=versions/uuid.uuid4().hex;shutil.copytree(source,version);validate(version)
        for f in version.iterdir():
            if f.is_file():
                with f.open('rb') as handle:os.fsync(handle.fileno())
        fd=os.open(version,os.O_RDONLY)
        try:os.fsync(fd)
        finally:os.close(fd)
        fd=os.open(versions,os.O_RDONLY)
        try:os.fsync(fd)
        finally:os.close(fd)
        maps=root/'maps';maps.mkdir(exist_ok=True);target=maps/'比赛地图'
        # One-time conversion of the old directory; later updates are one atomic rename.
        previous=None
        if target.exists() and not target.is_symlink():
            previous=versions/('previous-'+uuid.uuid4().hex);target.rename(previous)
        link=maps/('.competition-'+uuid.uuid4().hex)
        try:
            link.symlink_to(version,target_is_directory=True)
            os.replace(link,target)
        except OSError:
            link.unlink(missing_ok=True)
            if previous is not None and not target.exists():previous.rename(target)
            raise
        fd=os.open(maps,os.O_RDONLY)
        try:os.fsync(fd)
        finally:os.close(fd)
    return target/'room.yaml'

def resolve_map(root):
    root=Path(root).resolve()
    with locked(root):
        folder=(root/'maps/比赛地图').resolve(strict=True);validate(folder);return folder/'room.yaml'

if __name__=='__main__':
    root=Path(__file__).resolve().parent.parent
    if sys.argv[1]=='publish':print(publish(root,sys.argv[2]))
    elif sys.argv[1]=='resolve':print(resolve_map(root))
    else:raise SystemExit('Unknown map operation')
