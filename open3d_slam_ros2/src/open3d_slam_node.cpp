#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <open3d/Open3D.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <open3d_slam/SlamWrapper.hpp>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <open3d_slam/Mapper.hpp>
#include <open3d_slam/TransformInterpolationBuffer.hpp>
#include <mutex>
class Open3D_Slam_Node : public rclcpp::Node
{
public:
    o3d_slam::Time time;
    open3d::geometry::PointCloud point_cloud;

    Open3D_Slam_Node() : Node("open3d_slam_node")
    {
        // Topics
        this->declare_parameter<std::string>("cloud_topic", "/camera/depth/color/points");
        this->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
        this->declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
        this->declare_parameter<std::string>("camera_info_topic", "/camera/depth/camera_info");
        
        this->declare_parameter<std::string>("publish_cloud_topic", "my_point_cloud");
        this->declare_parameter<std::string>("trajectory_topic", "slam_trajectory");
        this->declare_parameter<std::string>("output_trajectory_file", "");

        slam_wrapper_ = std::make_shared<o3d_slam::SlamWrapper>();

        this->declare_parameter<std::string>("parameter_file", "");
        std::string parameter_file;
        this->get_parameter("parameter_file", parameter_file);
        if (!parameter_file.empty()) {
            slam_wrapper_->setParameterFilePath(parameter_file);
        }

        slam_wrapper_->loadParametersAndInitialize();
        slam_wrapper_->startWorkers();

        std::string cloud_topic_name, depth_topic_name, color_topic_name;
        std::string camera_info_topic_name, publish_cloud_topic, trajectory_topic;

        this->get_parameter("cloud_topic", cloud_topic_name);
        this->get_parameter("depth_topic", depth_topic_name);
        this->get_parameter("color_topic", color_topic_name);
        this->get_parameter("camera_info_topic", camera_info_topic_name);
        this->get_parameter("publish_cloud_topic", publish_cloud_topic);
        this->get_parameter("trajectory_topic", trajectory_topic);
        this->get_parameter("output_trajectory_file", trajectory_file_);

        // Publishers
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(publish_cloud_topic, 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(trajectory_topic, 10);

        // Subscribers
        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_name, 10,
            std::bind(&Open3D_Slam_Node::pointCloudCallback, this, std::placeholders::_1));

        depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic_name, 10,
            std::bind(&Open3D_Slam_Node::depthImageCallback, this, std::placeholders::_1));

        color_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            color_topic_name, 10,
            std::bind(&Open3D_Slam_Node::colorImageCallback, this, std::placeholders::_1));

        camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_name, 10,
            std::bind(&Open3D_Slam_Node::cameraInfoCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&Open3D_Slam_Node::publish_cloud, this));
    }

    ~Open3D_Slam_Node() {
        RCLCPP_INFO(this->get_logger(), "Shutting down... saving full trajectory.");
        saveFullTrajectory();
    }

private:

    std::mutex slam_mutex_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
    
    sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
    sensor_msgs::msg::Image::SharedPtr latest_color_image_; // Кеш для кольору
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<o3d_slam::SlamWrapper> slam_wrapper_;
    std::string trajectory_file_;
    
    nav_msgs::msg::Path robot_path_;
    o3d_slam::Time last_saved_time_;

    void saveFullTrajectory() {
        if (trajectory_file_.empty()) return;
        
        std::ofstream out(trajectory_file_);
        if (!out.is_open()) return;

        auto mapper = slam_wrapper_->getMapper(); 
        if (!mapper) return;

        const auto& buffer = mapper->getMapToRangeSensorBuffer();
        size_t n = buffer.size();
        if (n == 0) return;

        for (size_t i = 0; i < n; ++i) {
            int offset = n - 1 - i; 
            auto measurement = buffer.latest_measurement(offset);
            
            o3d_slam::Time t = measurement.time_; 
            Eigen::Isometry3d pose = measurement.transform_;
            
            auto duration_since_epoch = t.time_since_epoch();
            double timestamp_sec = std::chrono::duration<double>(duration_since_epoch).count();
            
            Eigen::Vector3d translation = pose.translation();
            Eigen::Quaterniond q(pose.rotation());
            
            out << std::fixed << std::setprecision(6) << timestamp_sec << " "
                << translation.x() << " " << translation.y() << " " << translation.z() << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        }
        out.close();
    }

    void publishLiveTrajectory() {
        auto T = slam_wrapper_->getLatestRegisteredCloudTimestampPair();
        if (T.second == o3d_slam::Time()) return;
        
        if (T.second == last_saved_time_) return;
        last_saved_time_ = T.second;

        Eigen::Isometry3d pose = slam_wrapper_->getPoseAt(T.second);
        Eigen::Vector3d translation = pose.translation();
        Eigen::Quaterniond q(pose.rotation());

        robot_path_.header.stamp = this->get_clock()->now(); 
        robot_path_.header.frame_id = "map"; 

        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.header = robot_path_.header;
        
        current_pose.pose.position.x = translation.x();
        current_pose.pose.position.y = translation.y();
        current_pose.pose.position.z = translation.z();
        
        current_pose.pose.orientation.x = q.x();
        current_pose.pose.orientation.y = q.y();
        current_pose.pose.orientation.z = q.z();
        current_pose.pose.orientation.w = q.w();

        robot_path_.poses.push_back(current_pose);
        path_publisher_->publish(robot_path_);
    }

    void publish_cloud() {
        std::lock_guard<std::mutex> lock(slam_mutex_);
        auto mapper = slam_wrapper_->getMapper();
        if (!mapper) return;

        open3d::geometry::PointCloud global_map = mapper->getAssembledMapPointCloud();
        size_t num_points = global_map.points_.size();

        if (num_points == 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Global map is empty");
            return;
        }

        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "map"; // або frame, на який налаштований твій TF tree

        sensor_msgs::PointCloud2Modifier modifier(*msg);
        bool has_colors = global_map.HasColors();
        
        if (has_colors) {

            modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        } else {
            modifier.setPointCloud2FieldsByString(1, "xyz");
        }
        modifier.resize(num_points);

        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
        
        if (has_colors) {

            sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(*msg, "rgb");
            
            for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
                *iter_x = global_map.points_[i].x();
                *iter_y = global_map.points_[i].y();
                *iter_z = global_map.points_[i].z();


                uint8_t r = static_cast<uint8_t>(std::clamp(global_map.colors_[i].x() * 255.0, 0.0, 255.0));
                uint8_t g = static_cast<uint8_t>(std::clamp(global_map.colors_[i].y() * 255.0, 0.0, 255.0));
                uint8_t b = static_cast<uint8_t>(std::clamp(global_map.colors_[i].z() * 255.0, 0.0, 255.0));

                iter_rgb[0] = b; // Blue
                iter_rgb[1] = g; // Green
                iter_rgb[2] = r; // Red

            } 
        } else {
            for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
                *iter_x = global_map.points_[i].x();
                *iter_y = global_map.points_[i].y();
                *iter_z = global_map.points_[i].z();
            }
        }
       
        publisher_->publish(std::move(msg));
    }
    void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        point_cloud = convertROSCloudToOpen3D(msg);
        if (point_cloud.IsEmpty()) {
            RCLCPP_WARN(this->get_logger(), "Cloud is empty");
            return;//Skip empty cloud
        }


        if (point_cloud.points_.size() < 100) {
            RCLCPP_WARN(this->get_logger(), "Cloud is too small (%zu points)", point_cloud.points_.size());
            return;
        }

        // Calculate normals if they are none
        if (!point_cloud.HasNormals()) {

            point_cloud.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
            point_cloud.OrientNormalsTowardsCameraLocation(Eigen::Vector3d(0.0, 0.0, 0.0));
        }
        time = fromROSTime(msg->header.stamp);


        slam_wrapper_->addRangeScan(point_cloud, time);
        publishLiveTrajectory();
    }


    void colorImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        latest_color_image_ = msg;
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        latest_camera_info_ = msg;
    }

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!latest_camera_info_) return;
        if (msg->encoding != "16UC1") return;

        open3d::geometry::PointCloud o3d_cloud;
        uint32_t width = msg->width;
        uint32_t height = msg->height;
        o3d_cloud.points_.reserve(width * height);

        const uint16_t* row_ptr = reinterpret_cast<const uint16_t*>(msg->data.data());
        size_t row_step = msg->step / sizeof(uint16_t);
        double fx = latest_camera_info_->k[0];
        double fy = latest_camera_info_->k[4];
        double cx = latest_camera_info_->k[2];
        double cy = latest_camera_info_->k[5];


        bool has_color = false;
        bool is_bgr = true;
        const uint8_t* color_row_ptr = nullptr;
        size_t color_row_step = 0;

        if (latest_color_image_) {
            double t_depth = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            double t_color = latest_color_image_->header.stamp.sec + latest_color_image_->header.stamp.nanosec * 1e-9;

            if (std::abs(t_depth - t_color) < 0.1) {
                if (latest_color_image_->encoding == "bgr8") {
                    has_color = true; is_bgr = true;
                } else if (latest_color_image_->encoding == "rgb8") {
                    has_color = true; is_bgr = false;
                }
                
                if (has_color) {
                    color_row_ptr = latest_color_image_->data.data();
                    color_row_step = latest_color_image_->step;
                    o3d_cloud.colors_.reserve(width * height);
                }
            }
        }

        for (uint32_t v = 0; v < height; ++v) {
            const uint16_t* ptr = row_ptr + v * row_step;
            const uint8_t* c_ptr = has_color ? color_row_ptr + v * color_row_step : nullptr;

            for (uint32_t u = 0; u < width; ++u) {
                uint16_t depth_mm = ptr[u];
                if (depth_mm == 0) continue;
                double z = static_cast<double>(depth_mm) * 0.001; 
                if (z <= 0 || z > 10.0) continue; 
                double x = (static_cast<double>(u) - cx) * z / fx;
                double y = (static_cast<double>(v) - cy) * z / fy;
                o3d_cloud.points_.emplace_back(x, y, z);

                if (has_color) {
                    int idx = u * 3;
                    double r, g, b;
                    if (is_bgr) {
                        b = static_cast<double>(c_ptr[idx + 0]) / 255.0;
                        g = static_cast<double>(c_ptr[idx + 1]) / 255.0;
                        r = static_cast<double>(c_ptr[idx + 2]) / 255.0;
                    } else {
                        r = static_cast<double>(c_ptr[idx + 0]) / 255.0;
                        g = static_cast<double>(c_ptr[idx + 1]) / 255.0;
                        b = static_cast<double>(c_ptr[idx + 2]) / 255.0;
                    }
                    o3d_cloud.colors_.emplace_back(r, g, b);
                }
            }
        }

        if (o3d_cloud.IsEmpty()) return;



        point_cloud = o3d_cloud;
        if (point_cloud.IsEmpty()) {
            RCLCPP_WARN(this->get_logger(), "Cloud is empty");
            return;
        }


        if (point_cloud.points_.size() < 100) {
            RCLCPP_WARN(this->get_logger(), "Cloud is too small (%zu points)", point_cloud.points_.size());
            return;
        }


        if (!point_cloud.HasNormals()) {

            point_cloud.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
            point_cloud.OrientNormalsTowardsCameraLocation(Eigen::Vector3d(0.0, 0.0, 0.0));
        }
        time = fromROSTime(msg->header.stamp);



        try {
            std::lock_guard<std::mutex> lock(slam_mutex_);
            slam_wrapper_->addRangeScan(point_cloud, time);
            publishLiveTrajectory();
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                 "SLAM processing error: %s", e.what());
        }
    }

    open3d::geometry::PointCloud convertROSCloudToOpen3D(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
        open3d::geometry::PointCloud o3d_cloud;
        o3d_cloud.points_.reserve(msg->width * msg->height);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        bool has_rgb = false;
        for (const auto& field : msg->fields) {
            if (field.name == "rgb" || field.name == "rgba") { has_rgb = true; break; }
        }

        if (has_rgb) {
            o3d_cloud.colors_.reserve(msg->width * msg->height);
            sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_rgb(*msg, "rgb");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
                if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
                    o3d_cloud.points_.emplace_back(*iter_x, *iter_y, *iter_z);
                    double b = static_cast<double>(iter_rgb[0]) / 255.0;
                    double g = static_cast<double>(iter_rgb[1]) / 255.0;
                    double r = static_cast<double>(iter_rgb[2]) / 255.0;
                    o3d_cloud.colors_.emplace_back(r, g, b);
                }
            }
        } else {
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
                    o3d_cloud.points_.emplace_back(*iter_x, *iter_y, *iter_z);
                }
            }
        }
        return o3d_cloud;
    }

    o3d_slam::Time fromROSTime(const builtin_interfaces::msg::Time& stamp) {
        uint64_t universal_ticks = (static_cast<uint64_t>(stamp.sec) * 10000000ull) + (stamp.nanosec / 100ull);
        return o3d_slam::fromUniversal(universal_ticks); 
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Open3D_Slam_Node>());
    rclcpp::shutdown();
    return 0;
}
