#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
import numpy as np
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
import small_gicp
import threading
import queue

DOWNSAMPLE_RESOLUTION = 0.05
NUM_NEIGHBORS         = 50
MAX_CORR_DIST         = 0.20
NUM_THREADS           = 4
MAX_ITERATIONS        = 50
VOXEL_LEAF_SIZE       = 0.05

class SmallGICPSLAM(Node):
    def __init__(self):
        super().__init__('small_gicp_slam')
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.current_pose = np.eye(4)
        self.estimated_poses = []
        self.timestamps = []
        self.ground_truth_poses = []
        self.frame_count = 0
        self.voxelmap = None
        self.depth_queue = queue.Queue()

        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.info_cb, 1000)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 1000)
        self.create_subscription(PoseStamped, '/camera_pose', self.gt_cb, 1000)

        self.worker = threading.Thread(target=self.process_loop, daemon=True)
        self.worker.start()
        self.get_logger().info('small_gicp SLAM node started (scan-to-map)')

    def info_cb(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_cb(self, msg):
        self.depth_queue.put(msg)

    def gt_cb(self, msg):
        self.ground_truth_poses.append(msg)

    def process_loop(self):
        while rclpy.ok():
            try:
                msg = self.depth_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            if self.fx is None:
                self.depth_queue.task_done()
                continue
            self.process_frame(msg)
            self.depth_queue.task_done()

    def process_frame(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth = depth.astype(np.float32) / 1000.0
        depth[depth < 0.1] = 0
        depth[depth > 10.0] = 0
        points = self.depth_to_pointcloud(depth)
        if points.shape[0] < 100:
            return

        pcd, _ = small_gicp.preprocess_points(
            points,
            downsampling_resolution=DOWNSAMPLE_RESOLUTION,
            num_neighbors=NUM_NEIGHBORS,
            num_threads=NUM_THREADS
        )

        if self.voxelmap is None:
            self.voxelmap = small_gicp.GaussianVoxelMap(VOXEL_LEAF_SIZE)
            self.voxelmap.insert(pcd, self.current_pose)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.timestamps.append(t)
            self.estimated_poses.append(self.current_pose.copy())
            self.frame_count += 1
            return

        try:
            result = small_gicp.align(
                self.voxelmap, pcd,
                init_T_target_source=self.current_pose,
                max_correspondence_distance=MAX_CORR_DIST,
                num_threads=NUM_THREADS,
                max_iterations=MAX_ITERATIONS
            )
            self.current_pose = result.T_target_source
            self.voxelmap.insert(pcd, self.current_pose)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.timestamps.append(t)
            self.estimated_poses.append(self.current_pose.copy())
            self.frame_count += 1

            if self.frame_count % 100 == 0:
                self.get_logger().info(f'processed {self.frame_count} frames | queue: {self.depth_queue.qsize()} pending')

        except Exception as e:
            self.get_logger().warn(f'alignment failed: {e}')

    def depth_to_pointcloud(self, depth):
        h, w = depth.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        mask = depth > 0
        points = np.stack([x[mask], y[mask], depth[mask]], axis=-1)
        return points.astype(np.float32)

    def save_results(self):
        self.get_logger().info(f'waiting for {self.depth_queue.qsize()} remaining frames...')
        self.depth_queue.join()
        with open('/home/vignesh-menon/estimated_tum.txt', 'w') as f:
            for t, pose in zip(self.timestamps, self.estimated_poses):
                tx, ty, tz = pose[:3, 3]
                qx, qy, qz, qw = Rotation.from_matrix(pose[:3, :3]).as_quat()
                f.write(f'{t:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n')
        with open('/home/vignesh-menon/groundtruth_tum.txt', 'w') as f:
            for msg in self.ground_truth_poses:
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                p = msg.pose.position
                o = msg.pose.orientation
                f.write(f'{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {o.x:.6f} {o.y:.6f} {o.z:.6f} {o.w:.6f}\n')
        self.get_logger().info(f'saved {len(self.estimated_poses)} estimated and {len(self.ground_truth_poses)} ground truth poses')

def main():
    rclpy.init()
    node = SmallGICPSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save_results()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
