#!/usr/bin/python3
"""Remove staging paths and give packaged ELF files relative library paths."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess


def relocate(prefix, patchelf):
    prefix = prefix.resolve()
    for file in prefix.rglob('*'):
        if not file.is_file() or file.is_symlink():
            continue
        with file.open('rb') as stream:
            magic = stream.read(4)
        if magic != b'\x7fELF':
            continue
        paths = [prefix / 'libexec/sawyer/ros1/lib', prefix / 'lib']
        rpath = ':'.join('$ORIGIN/' + os.path.relpath(path, file.parent) for path in paths)
        rpath += ':/opt/ros/humble/lib'
        subprocess.run([str(patchelf), '--set-rpath', rpath, str(file)], check=True)
    # Consumers source local_setup.bash, whose prefix is derived from its path.
    # setup.* chain scripts capture the build prefix, so omit those redundant files.
    for name in ('setup.bash', 'setup.sh', 'setup.zsh'):
        (prefix / name).unlink(missing_ok=True)
    print('Applied relative ELF paths under', prefix)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', type=Path)
    parser.add_argument('--patchelf', type=Path, default=shutil.which('patchelf'))
    args = parser.parse_args()
    if not args.patchelf:
        parser.error('Install patchelf in the package-building tools prefix')
    relocate(args.prefix, args.patchelf)
