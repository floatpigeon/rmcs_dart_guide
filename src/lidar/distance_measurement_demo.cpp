#include "point_cloud_process.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <pcl/conversions.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/file_io.h>
#include <pcl/io/pcd_io.h>
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
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>

namespace rmcs_dart_guide {

using PointT      = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class DistanceMeasureDemo
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DistanceMeasureDemo()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", rclcpp::SystemDefaultsQoS(),
            std::bind(&DistanceMeasureDemo::lidar_data_callback, this, std::placeholders::_1));

        pointcloud_publisher1_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/dart_guidance/output_pointcloud1", 100);

        pointcloud_publisher2_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/dart_guidance/output_pointcloud2", 100);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        distance_measure_thread_ = std::thread(&DistanceMeasureDemo::distance_measure, this);
    }

    void update() override {}

private:
    void init() {
        integral_frame_count_ = 0;
        measure_start_flag_   = false;
        std::lock_guard<std::mutex> lock(distance_measure_thread_mtx_);
        integral_pointcloud_ = std::make_shared<PointCloudT>();
    }

    void lidar_data_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // RCLCPP_INFO(logger_, "received message");
        PointCloudT::Ptr received_cloud(new PointCloudT());
        pcl::fromROSMsg(*msg, *received_cloud);

        {
            std::lock_guard<std::mutex> lock(distance_measure_thread_mtx_);
            if (integral_frame_count_ == 0) {
                integral_pointcloud_ = received_cloud;
            } else {
                *integral_pointcloud_ += *received_cloud;
            }
        }
        integral_frame_count_++;

        if (integral_frame_count_ == 20) {
            measure_start_flag_ = true;
            RCLCPP_INFO(logger_, "start measure");
        }
    }

    PointCloudT::Ptr integral_pointcloud_;
    std::atomic<int> integral_frame_count_ = 0;
    std::atomic<bool> measure_start_flag_  = false;
    std::thread distance_measure_thread_;
    std::mutex distance_measure_thread_mtx_;

    std::string target_filename_ =
        "/workspaces/RMCS/rmcs_ws/src/rmcs_dart_guide/src/point_cloud_files/milkdragon_test_target.pcd";

    std::string scence_filename_ =
        "/workspaces/RMCS/rmcs_ws/src/rmcs_dart_guide/src/point_cloud_files/milkdragon_test_pointcloud.pcd";

    void target_cloud_process(PointCloudT::Ptr& output_target_cloud) {
        PointCloudT::Ptr target_cloud = std::make_shared<PointCloudT>();
        auto return_status            = pcl::io::loadPCDFile(target_filename_, *target_cloud);

        if (return_status == 0) {
            RCLCPP_INFO(this->get_logger(), "Successfully load %s", target_filename_.c_str());
        } else {
            RCLCPP_ERROR(
                this->get_logger(), "Failed to load %s (Error code: %d)", target_filename_.c_str(), return_status);
        }

        pcl::VoxelGrid<pcl::PointXYZ> grid_filter;
        grid_filter.setInputCloud(target_cloud);
        grid_filter.setLeafSize(0.005f, 0.005f, 0.005f);
        grid_filter.filter(*output_target_cloud);
    }

    void distance_measure() {
        while (true) {
            if (measure_start_flag_ == false) {
                RCLCPP_INFO(logger_, "not enough data");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            PointCloudT::Ptr original_cloud = std::make_shared<PointCloudT>();
            {
                std::lock_guard<std::mutex> lock(distance_measure_thread_mtx_);
                original_cloud = integral_pointcloud_;
            }
            init();

            // PointCloudT::Ptr original_cloud = std::make_shared<PointCloudT>();
            // pcl::io::loadPCDFile(scence_filename_, *original_cloud);

            // 降采样
            PointCloudT::Ptr source_cloud = std::make_shared<PointCloudT>();
            PointCloudProcess::box_filter_and_downsample(original_cloud, source_cloud);

            // 获取目标点云
            PointCloudT::Ptr target_cloud = std::make_shared<PointCloudT>();
            target_cloud_process(target_cloud);
            pointcloud_data_publish(target_cloud, pointcloud_publisher1_);

            // 特征获取
            pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features =
                std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>(
                    PointCloudProcess::get_features(source_cloud));

            pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features =
                std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>(
                    PointCloudProcess::get_features(target_cloud));

            pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
            PointCloudProcess::match_features(target_features, source_features, correspondences);

            pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> est;
            Eigen::Matrix4f initial_guess_transformation; // 初始猜测
            est.estimateRigidTransformation(
                *source_cloud, *target_cloud, *correspondences, initial_guess_transformation);

            fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;
            gicp.setTransformationEpsilon(1e-12);
            gicp.setMaximumIterations(100);
            gicp.setInputSource(source_cloud);
            gicp.setInputTarget(target_cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud = std::make_shared<PointCloudT>();
            gicp.align(*final_cloud, initial_guess_transformation);
            pointcloud_data_publish(final_cloud, pointcloud_publisher2_);

            // pointcloud_data_publish(source_cloud);

            if (gicp.hasConverged()) {
                RCLCPP_INFO(logger_, "收敛,score:%f", gicp.getFitnessScore());
            } else {
                RCLCPP_INFO(logger_, "未收敛,score:%f", gicp.getFitnessScore());
            }

            Eigen::Vector4f original_source_centroid_vector;
            pcl::compute3DCentroid(*source_cloud, original_source_centroid_vector); // 注意这里是 source_cloud
            Eigen::Vector3f distance_vector(
                original_source_centroid_vector.x(), original_source_centroid_vector.y(),
                original_source_centroid_vector.z());
            double distance = distance_vector.norm();
            RCLCPP_INFO(logger_, "distance:%f", distance);
        }
    }

    void pointcloud_data_publish(
        PointCloudT::Ptr& pointcloud, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) {
        // 点云和tf需要统一的时间
        rclcpp::Time message_time_stamped = this->get_clock()->now();

        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(*pointcloud, point_cloud_msg);

        point_cloud_msg.header.stamp    = message_time_stamped;
        point_cloud_msg.header.frame_id = "lidar";
        publisher->publish(point_cloud_msg);

        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp    = message_time_stamped;
        transform_stamped.header.frame_id = "world";
        transform_stamped.child_frame_id  = "lidar";

        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 1.0;

        // 测距过程中保持静止，不考虑旋转
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);

        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    rclcpp::Logger logger_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher1_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher2_;
};
} // namespace rmcs_dart_guide

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guide::DistanceMeasureDemo, rmcs_executor::Component)