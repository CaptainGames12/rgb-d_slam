#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <Eigen/Dense>

class ICPNode : public rclcpp::Node
{
public:
  ICPNode() : Node("icp_node"), first_frame_(true)
  {
    // Subscriptions
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", 1,
      std::bind(&ICPNode::depthCallback, this, std::placeholders::_1));

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/depth/camera_info", 10,
      std::bind(&ICPNode::infoCallback, this, std::placeholders::_1));

    // Publisher
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/estimated_pose", 10);

    // Start with identity pose
    current_pose_ = Eigen::Matrix4f::Identity();

    RCLCPP_INFO(get_logger(), "ICP Node started");
  }

private:
  // Store camera intrinsics when received
  void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    has_intrinsics_ = true;
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!has_intrinsics_) {
      RCLCPP_WARN(get_logger(), "Waiting for intrinsics...");
      return;
    }

    // Convert ROS depth image to OpenCV
    cv_bridge::CvImagePtr cv_img;
    try {
      cv_img = cv_bridge::toCvCopy(msg, "16UC1");
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    // Convert depth image to point cloud
    auto cloud = depthToPointCloud(cv_img->image);
    RCLCPP_INFO(get_logger(), "Cloud size: %zu points", cloud->points.size());

    if (first_frame_) {
      prev_cloud_ = cloud;
      first_frame_ = false;
      RCLCPP_INFO(get_logger(), "First frame stored");
      return;
    }

    // Run ICP between previous and current cloud
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud);
    icp.setInputTarget(prev_cloud_);
    icp.setMaximumIterations(2);
    icp.setTransformationEpsilon(1e6);
    icp.setMaxCorrespondenceDistance(0.3);
    icp.setEuclideanFitnessEpsilon(1e4);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned);

    if (icp.hasConverged()) {
      // Accumulate the pose
      Eigen::Matrix4f delta = icp.getFinalTransformation();
      current_pose_ = current_pose_ * delta;

      // Publish the pose
      publishPose(msg->header.stamp);
      RCLCPP_INFO(get_logger(), "ICP converged, score: %.6f", icp.getFitnessScore());
    } else {
      RCLCPP_WARN(get_logger(), "ICP did not converge");
    }

    prev_cloud_ = cloud;
  }

  // Convert a 16-bit depth image to a PCL point cloud using camera intrinsics
  pcl::PointCloud<pcl::PointXYZ>::Ptr depthToPointCloud(const cv::Mat & depth)
  {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    for (int v = 0; v < depth.rows; v += 2) {
      for (int u = 0; u < depth.cols; u += 2) {
        uint16_t raw = depth.at<uint16_t>(v, u);
        if (raw == 0) continue;  // no measurement

        float z = raw / 1000.0f;  // millimetres → metres
        float x = (u - cx_) * z / fx_;
        float y = (v - cy_) * z / fy_;

        cloud->points.emplace_back(x, y, z);
      }
    }

   cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    // Downsample to speed up ICP
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.35f, 0.35f, 0.35f);  // 5cm voxel size
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    voxel.filter(*filtered);

    return filtered;
  }

  void publishPose(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "map";

    // Extract translation
    pose_msg.pose.position.x = current_pose_(0, 3);
    pose_msg.pose.position.y = current_pose_(1, 3);
    pose_msg.pose.position.z = current_pose_(2, 3);

    // Extract rotation as quaternion
    Eigen::Matrix3f rot = current_pose_.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rot);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_pub_->publish(pose_msg);
  }

  // Subscriptions & publisher
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // State
  pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
  Eigen::Matrix4f current_pose_;
  bool first_frame_;

  // Intrinsics
  double fx_, fy_, cx_, cy_;
  bool has_intrinsics_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ICPNode>());
  rclcpp::shutdown();
  return 0;
}