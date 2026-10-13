"""Fetch the pinned native ROS 1 actionlib implementation for fixture tests only."""
import hashlib
import json
from pathlib import Path
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
spec = json.loads((ROOT / 'packaging/test-sources.json').read_text())['actionlib']
archive = ROOT / '.runtime/pkg-sources/actionlib.tar.gz'
archive.parent.mkdir(parents=True, exist_ok=True)
if not archive.exists():
    with urllib.request.urlopen(spec['url'], timeout=30) as response:
        archive.write_bytes(response.read())
assert hashlib.sha256(archive.read_bytes()).hexdigest() == spec['sha256'], 'actionlib checksum mismatch'
target = ROOT / '.runtime/actionlib-fixture'
target.mkdir(parents=True, exist_ok=True)
with tarfile.open(archive) as source:
    for member in source.getmembers():
        if member.issym() or member.islnk():
            continue
        destination = (target / member.name).resolve()
        assert destination.is_relative_to(target.resolve()), 'Unsafe source archive path'
        source.extract(member, target)
print(target / ('actionlib-' + spec['version']) / 'actionlib/src')
