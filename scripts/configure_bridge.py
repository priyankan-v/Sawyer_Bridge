#!/usr/bin/env python3
"""Load selective bridge settings into ROS 1; never publish robot commands."""
import argparse
from pathlib import Path

import rospy
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    parser.add_argument('--commands', action='store_true')
    parser.add_argument('--head-commands', action='store_true')
    parser.add_argument('--gripper-commands', action='store_true')
    parser.add_argument('--cameras', action='store_true')
    args = parser.parse_args()
    with args.config.open() as stream:
        config = yaml.safe_load(stream)
    for entry in config['topics']:
        if args.gripper_commands and entry['topic'] == '/io/end_effector/right_gripper/command':
            entry['enabled'] = True
        if args.commands and entry['topic'] in (
            '/robot/limb/right/joint_command', '/robot/limb/right/joint_command_timeout',
            '/robot/limb/right/set_speed_ratio'
        ):
            entry['enabled'] = True
        if args.head_commands and entry['topic'] in (
            '/robot/head/head_state', '/robot/head/command_head_pan'
        ):
            entry['enabled'] = True
        if args.cameras and entry['topic'] in (
            '/io/internal_camera/head_camera/image_rect_color',
            '/io/internal_camera/right_hand_camera/image_rect'
        ):
            entry['enabled'] = True
    rospy.init_node('configure_sawyer_bridge', anonymous=True)
    for key in ('topics', 'services_1_to_2', 'services_2_to_1'):
        rospy.set_param('/sawyer_bridge/' + key, config.get(key, []))


if __name__ == '__main__':
    main()
