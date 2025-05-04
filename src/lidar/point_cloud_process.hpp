#pragma once

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <pcl/common/transforms.h>
#include <pcl/correspondence.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/search/kdtree.h>

namespace rmcs_dart_guide {

class PointCloudProcess {
public:
    static void box_filter_and_downsample(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr box_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setInputCloud(input);

        // 基地大致距离
        // Eigen::Vector4f min_point(25.0, -2.5, -0.5, 1.0);
        // Eigen::Vector4f max_point(30, 2.5, 0.5, 1.0);

        // // TEST
        Eigen::Vector4f min_point(0.0, -2.5, -0.75, 1.0);
        Eigen::Vector4f max_point(3.0, 2.5, 0.75, 1.0);

        box_filter.setMin(min_point);
        box_filter.setMax(max_point);
        box_filter.setNegative(false);
        box_filter.filter(*box_filtered_cloud);

        pcl::VoxelGrid<pcl::PointXYZ> grid_filter;
        grid_filter.setInputCloud(box_filtered_cloud);
        grid_filter.setLeafSize(0.01f, 0.01f, 0.01f);
        grid_filter.filter(*output);
    }

    static pcl::PointCloud<pcl::FPFHSignature33> get_features(pcl::PointCloud<pcl::PointXYZ>::Ptr& input) {
        // TODO:从AI改的，需review
        // 什么法线特征协方差没太看明白，先把功能实现了再说（20250503）

        // 法线估计
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        normal_estimation.setInputCloud(input);
        normal_estimation.setSearchMethod(tree);
        normal_estimation.setRadiusSearch(0.15);
        normal_estimation.compute(*cloud_normals);

        // 特征提取也需要一个 KdTree，搜索半径可能与法线估计不同
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fpfh(new pcl::search::KdTree<pcl::PointXYZ>());
        tree_fpfh->setInputCloud(input);

        // 提取FPFH特征
        pcl::PointCloud<pcl::FPFHSignature33> fpfh_features;
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
        fpfh_est.setInputCloud(input);
        fpfh_est.setInputNormals(cloud_normals);
        fpfh_est.setSearchMethod(tree_fpfh);
        fpfh_est.setRadiusSearch(0.3);
        fpfh_est.compute(fpfh_features);

        return fpfh_features;
    }

    static void match_features(
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr& cloud_target_features,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr& cloud_scence_features,
        pcl::CorrespondencesPtr& correspondences) {

        // TODO:从AI改的，需review

        std::cout << "目标特征点云包含 " << cloud_target_features->size() << " 个特征." << std::endl;
        std::cout << "场景特征点云包含 " << cloud_scence_features->size() << " 个特征." << std::endl;

        // --- 2. 为目标特征点云构建 KdTree ---
        // KdTreeFLANN 通常比 KdTree 更快，特别是对于浮点数类型
        pcl::KdTreeFLANN<pcl::FPFHSignature33> kdtree_features;
        kdtree_features.setInputCloud(cloud_target_features);

        // --- 3. 进行特征匹配 ---

        float max_distance = std::numeric_limits<float>::max(); // 初始化最大距离

        // 遍历场景点云中的每一个特征
        for (size_t i = 0; i < cloud_scence_features->points.size(); ++i) {
            std::vector<int> neighbor_indices(2);             // 搜索 2 个最近邻 (用于比率测试)
            std::vector<float> neighbor_squared_distances(2); // 存储距离的平方

            // 在目标特征的 KdTree 中搜索当前场景特征的最近邻
            if (kdtree_features.nearestKSearch(
                    cloud_scence_features->points[i], 2, neighbor_indices, neighbor_squared_distances)
                > 0) {
                // 执行比率测试
                // 如果最近邻距离的平方 与 次近邻距离的平方 的比率小于阈值
                float ratio_threshold = 0.7; // 可以根据数据调整这个阈值
                if (neighbor_squared_distances[0] < ratio_threshold * neighbor_squared_distances[1]) {
                    // 认为这是一个有效的匹配
                    pcl::Correspondence corr(
                        static_cast<int>(i), neighbor_indices[0], neighbor_squared_distances[0]);
                    correspondences->push_back(corr);
                }
                // 可以选择记录最大的最近邻距离，用于后续可能的全局阈值
                if (neighbor_squared_distances[0] < max_distance)
                    max_distance = neighbor_squared_distances[0];
            }
        }

        std::cout << "找到 " << correspondences->size() << " 对特征匹配." << std::endl;
    }

    static void pose_estimation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        const pcl::CorrespondencesPtr& correspondences) {
        // 这个函数可以直接放在回调函数里面去

        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> est;
        Eigen::Matrix4f estimated_transformation; // 初始猜测

        est.estimateRigidTransformation(*source_cloud, *target_cloud, *correspondences, estimated_transformation);

        fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(source_cloud);
        gicp.setInputTarget(target_cloud);

        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        Eigen::Matrix4f transform;
        gicp.align(final_cloud, estimated_transformation);
    }
};

class PointCloudIntegral {
public:
    PointCloudIntegral() {
        integral_pointcloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }

    void add_pointcloudframe(pcl::PointCloud<pcl::PointXYZ>::Ptr& input) { *integral_pointcloud_ += *input; }

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr integral_pointcloud_;
};

} // namespace rmcs_dart_guide