#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <memory>
#include <vector>

namespace metro {

struct DetectorConfig {
    float cluster_tolerance = 0.6f;
    int min_cluster_size = 6;
    int max_cluster_size = 100000;
    float min_obstacle_height = 0.15f;
    float max_obstacle_length = 5.0f;
    float max_obstacle_width = 2.7f;
    float density_coefficient = 900.0f;
};

class IClusterStrategy {
public:
    virtual ~IClusterStrategy() = default;
    virtual std::vector<pcl::PointIndices> extract(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const = 0;
};

} // namespace metro
