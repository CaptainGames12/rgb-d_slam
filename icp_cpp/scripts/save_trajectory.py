#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TrajectorySaver(Node):
    def __init__(self):
        super().__init__('trajectory_saver')
        self.file = open('/home/swayam/estimated_trajectory.txt', 'w')
        self.subscription = self.create_subscription(
            PoseStamped, '/estimated_pose', self.callback, 10)
        self.get_logger().info('Saving trajectory...')

    def callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        q = msg.pose.orientation
        self.file.write(f"{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
                       f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = TrajectorySaver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
