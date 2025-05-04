#include "point_cloud_process.hpp"
#include <memory>
#include <mutex>
#include <pcl/conversions.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>

namespace rmcs_dart_guide {

class LidarController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    LidarController()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        pcl_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", rclcpp::SystemDefaultsQoS(),
            std::bind(&LidarController::lidar_data_callback, this, std::placeholders::_1));

        pointcloud_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/dart_guidance/processed_pointcloud", 100);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    void update() override {}

private:
    void lidar_data_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        message_time_stamped = this->get_clock()->now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *original_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        PointCloudProcess::box_filter_and_downsample(original_cloud, filtered_cloud);

        sensor_msgs::msg::PointCloud2 point_cloud_msg;

        pcl::toROSMsg(*filtered_cloud, point_cloud_msg);
        point_cloud_msg.header.stamp    = message_time_stamped;
        point_cloud_msg.header.frame_id = "lidar";
        pointcloud_publisher_->publish(point_cloud_msg);

        lidar_coordinate_axis_publish();

        RCLCPP_INFO(
            logger_, "point cloud received,num: %d,filterd num: %d", msg->width * msg->height,
            point_cloud_msg.width * point_cloud_msg.height);
    }

    void lidar_coordinate_axis_publish() {
        // === 构造并发布 TF 变换 ===
        // 这个变换描述了点云所在的坐标系 ("lidar") 相对于 RViz Fixed Frame ("world") 的位置和姿态。
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp    = message_time_stamped; // 和点云使用相同的时间戳
        transform_stamped.header.frame_id = "world";              // 父坐标系 (RViz 通常以此为 Fixed Frame)
        transform_stamped.child_frame_id  = "lidar";              // 子坐标系 (点云的 frame_id)

        // 设置平移 (Translate "lidar" relative to "world")
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 1.0;

        // 设置旋转 (Rotate "lidar" relative to "world")
        tf2::Quaternion q;
        q.setRPY(0, 0, 0); // 没有旋转 (Roll, Pitch, Yaw 都为 0)
        // 如果雷达面向其他方向，你需要设置相应的 RPY 或四元数

        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        // 发布 TF 变换
        tf_broadcaster_->sendTransform(transform_stamped);
    }

    rclcpp::Logger logger_;

    rclcpp::Time message_time_stamped;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_;

    void lidar_data_callback_beta(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *original_cloud);
        lidar_pointcloud_queue_.push_back(original_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        if (lidar_pointcloud_queue_.size() == 10) {
            measure_begin_flag_ = true;
        }

        PointCloudProcess::box_filter_and_downsample(original_cloud, filtered_cloud);

        // 发布部分
        message_time_stamped = this->get_clock()->now();
        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(*filtered_cloud, point_cloud_msg);
        point_cloud_msg.header.stamp    = message_time_stamped;
        point_cloud_msg.header.frame_id = "lidar";
        pointcloud_publisher_->publish(point_cloud_msg);

        lidar_coordinate_axis_publish();

        RCLCPP_INFO(
            logger_, "point cloud received,num: %d,filterd num: %d", msg->width * msg->height,
            point_cloud_msg.width * point_cloud_msg.height);
    }

    void distance_measure() {
        while (true) {
            if (measure_begin_flag_ == false)
                continue;

            pcl::PointCloud<pcl ::PointXYZ>::Ptr integral_pointcloud =
                std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

            pcl::PointCloud<pcl ::PointXYZ>::Ptr filtered_pointcloud =
                std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

            PointCloudProcess::box_filter_and_downsample(integral_pointcloud, filtered_pointcloud);
        }
    }

    pcl::PointCloud<pcl ::PointXYZ>::Ptr integral_pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> lidar_pointcloud_queue_;

    std::atomic<bool> measure_begin_flag_ = false;
    std::thread distance_measure_thread_;
    std::mutex distance_measure_thread_mtx;
};
} // namespace rmcs_dart_guide

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guide::LidarController, rmcs_executor::Component)