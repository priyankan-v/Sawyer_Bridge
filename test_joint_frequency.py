#!/usr/bin/env python3

import time
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class JointFrequencyTest(Node):

    def __init__(self):
        super().__init__('joint_frequency_test')

        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.subscription = self.create_subscription(
            JointState,
            '/robot/joint_states',
            self.callback,
            qos
        )

        self.last_time = None
        self.intervals = []
        self.total_messages = 0

        self.timer = self.create_timer(5.0, self.report)

    def callback(self, msg):

        now = time.monotonic()

        if self.last_time is not None:
            self.intervals.append(now - self.last_time)

        self.last_time = now
        self.total_messages += 1

    def report(self):

        if len(self.intervals) < 2:
            print("Waiting for messages...")
            return

        avg_interval = statistics.mean(self.intervals)
        frequency = 1.0 / avg_interval

        print("\n--- ROS 2 Joint State Frequency ---")
        print(f"Average frequency : {frequency:.2f} Hz")
        print(f"Minimum interval  : {min(self.intervals)*1000:.2f} ms")
        print(f"Maximum interval  : {max(self.intervals)*1000:.2f} ms")
        print(f"Standard deviation: {statistics.pstdev(self.intervals)*1000:.2f} ms")
        print(f"Total messages    : {self.total_messages}")

        self.intervals.clear()


def main():
    rclpy.init()
    node = JointFrequencyTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
