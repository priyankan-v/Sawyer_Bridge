"""Run source-only ROS 1 test tools on localhost; never part of the package."""
import os
from pathlib import Path
import runpy
import sys
from urllib.parse import urlparse

assert urlparse(os.environ['ROS_MASTER_URI']).hostname in ('127.0.0.1', 'localhost')
assert os.environ['ROS_IP'] == '127.0.0.1'
root = Path(__file__).resolve().parents[1]
source = root / '.runtime/pkg-build/sources/ros_comm/ros_comm-1.17.4/tools'
tool = sys.argv.pop(1)
if tool == 'roscore':
    sys.path.insert(0, str(source / 'rosmaster/src'))
    from rosmaster.main import rosmaster_main
    sys.argv[0] = 'rosmaster'
    rosmaster_main()
elif tool == 'rostopic':
    sys.path.insert(0, str(source / 'rostopic/src'))
    import rostopic
    # The regression suite only needs the upstream publish command. The full
    # CLI imports rosbag even for publishing; bag support is not a dependency.
    if sys.argv[1] != 'pub':
        raise ValueError('This source-only test helper supports rostopic pub only')
    rostopic._rostopic_cmd_pub(sys.argv)
else:
    raise ValueError(tool)
