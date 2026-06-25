#!/usr/bin/env python3

import sys
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GroundTruthSaver(Node):

    def __init__(self, output_file):
        super().__init__('groundtruth_saver')

        os.makedirs(os.path.dirname(output_file), exist_ok=True)

        self.file = open(output_file, "w")

        self.subscription = self.create_subscription(
            PoseStamped,
            "/camera_pose",
            self.callback,
            10
        )

        self.get_logger().info(f"Saving ground truth to:\n{output_file}")

    def callback(self, msg):

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        p = msg.pose.position
        q = msg.pose.orientation

        self.file.write(
            f"{t:.6f} "
            f"{p.x:.6f} {p.y:.6f} {p.z:.6f} "
            f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"
        )

        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main():

    if len(sys.argv) != 2:
        print("Usage:")
        print("python3 save_groundtruth.py output_file.txt")
        return

    rclpy.init()

    node = GroundTruthSaver(sys.argv[1])

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()