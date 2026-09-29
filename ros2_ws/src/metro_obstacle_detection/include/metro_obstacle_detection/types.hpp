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

    // Hard cap на вход кластеризации: выше — аварийное прореживание,
    // иначе плотное облако (стена в габарите) роняет экстрактор по bad_alloc.
    int max_points_for_clustering = 20000;
    float emergency_leaf_size = 0.25f;
    // Потолок на точки публикуемого obstacle_points (защита toROSMsg).
    int max_publish_points = 30000;
};

class IClusterStrategy {
public:
    virtual ~IClusterStrategy() = default;
    virtual std::vector<pcl::PointIndices> extract(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const = 0;
};

} // namespace metro
