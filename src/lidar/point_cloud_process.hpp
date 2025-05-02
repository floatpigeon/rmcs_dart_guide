#pragma once

#include <pcl/filters/passthrough.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>

namespace rmcs_dart_guide {

class PointCloudProcess {
public:
    static void ground_point_filter(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) {
        pcl::PassThrough<pcl::PointXYZ> filter;

        filter.setInputCloud(input_cloud);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(-1.0, 1.0);
        filter.filter(*output_cloud);
    }
};

} // namespace rmcs_dart_guide