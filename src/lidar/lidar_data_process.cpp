#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace rmcs_dart_guide {

class LidarDataProcess
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    LidarDataProcess()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        received_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", rclcpp::SystemDefaultsQoS(),
            std::bind(&LidarDataProcess::lidar_data_callback, this, std::placeholders::_1));
    }

    void update() override {}

private:
    void lidar_data_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(logger_, "received,Number of points: %d", msg->width * msg->height);
        pcl::fromROSMsg(*msg, *received_cloud_);

        if (!received_cloud_->points.empty()) {
            const auto& first_point = received_cloud_->points[0];
            RCLCPP_INFO(logger_, "First point: (%f,%f,%f)", first_point.x, first_point.y, first_point.z);
        } else {
            RCLCPP_INFO(logger_, "point cloud empty");
        }
    }

    rclcpp::Logger logger_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr received_cloud_;
};
} // namespace rmcs_dart_guide

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guide::LidarDataProcess, rmcs_executor::Component)