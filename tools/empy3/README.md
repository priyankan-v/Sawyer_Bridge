# EmPy generator compatibility

The Conda environments include EmPy 4.2.1, while the legacy Noetic/Humble
message generators require **EmPy 3.3.4**.

This unmodified pure-Python module runs with Conda Python 3.12.
`scripts/env.sh` prepends this directory to `PYTHONPATH`;
`scripts/python_noetic.sh` preserves that ordering inside catkin's cached
build environment. Keep both for interface generation.

The source was copied from Ubuntu's `python3-empy` package. Its copyright and
license are retained in `COPYRIGHT` and `LICENSE`.
