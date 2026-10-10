#!/usr/bin/env python3
"""Run from pixi run ros2-shell; without --execute this only displays a plan."""
from sawyer_ros2_control.move_joint import main

if __name__ == '__main__':
    raise SystemExit(main())
