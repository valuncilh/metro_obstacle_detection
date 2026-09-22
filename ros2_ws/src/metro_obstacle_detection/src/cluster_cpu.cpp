#include "metro_obstacle_detection/types.hpp"
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace metro {

class CpuClusterStrategy : public IClusterStrategy {
public:
    explicit CpuClusterStrategy(const DetectorConfig& cfg) : cfg_(cfg) {}

    std::vector<pcl::PointIndices> extract(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const override {
        
        std::vector<pcl::PointIndices> cluster_indices;
        if (cloud->empty()) return cluster_indices;

        auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
        tree->setInputCloud(cloud);
        tree->setSortedResults(false);

        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(cfg_.cluster_tolerance);
        ec.setMinClusterSize(cfg_.min_cluster_size);
        ec.setMaxClusterSize(cfg_.max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        return cluster_indices;
    }

private:
    DetectorConfig cfg_;
};

std::unique_ptr<IClusterStrategy> create_cpu_strategy(const DetectorConfig& cfg) {
    return std::make_unique<CpuClusterStrategy>(cfg);
}

} // namespace metro
