#!/usr/bin/env python3
"""Read-only JointState reception records for frequency/loss correlation."""
import json
import sys
import time
from pathlib import Path
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('sawyer_readonly_ros1_measurement',anonymous=True)
rows=[]
rospy.Subscriber('/robot/joint_states',JointState,lambda m:rows.append({'received_ns':time.monotonic_ns(),
                  'stamp_ns':m.header.stamp.to_nsec(),'names':list(m.name),'sequence':m.header.seq}),queue_size=1000,tcp_nodelay=True)
time.sleep(float(sys.argv[2]) if len(sys.argv)>2 else 12)
Path(sys.argv[1]).write_text(json.dumps(rows)+'\n')
print(f'Captured {len(rows)} ROS 1 JointState messages.')
