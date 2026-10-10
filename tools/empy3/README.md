EmPy 3.3.4 is required by the unmodified ROS 1 `genmsg` templates and older
ROS 2 generator code. Pixi's locked environments supply EmPy 4.2.1.

This unmodified, pure-Python module was copied from the workstation's Ubuntu
`python3-empy` package (`/usr/lib/python3/dist-packages/em.py`). It runs under
Pixi Python 3.12; no Python 3.10 extension or site-packages directory is used.
`scripts/env.sh` places only this directory ahead of Pixi Python modules.
System Humble uses its original Python 3.10 runtime and EmPy.

The module's source, copyright and license are retained here. See `COPYRIGHT`
and `LICENSE`. The attempt to pin EmPy through Pixi failed because the package
index DNS was unavailable; this local compatibility dependency avoids changing
either installed environment or the existing lockfile.
