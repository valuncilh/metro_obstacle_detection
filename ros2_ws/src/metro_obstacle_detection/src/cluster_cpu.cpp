#include "metro_obstacle_detection/types.hpp"
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace metro {

class CpuClusterStrategy : public IClusterStrategy {
public:
    explicit CpuClusterStrategy(const DetectorConfig& cfg) : cfg_(cfg) {
        tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
        tree_->setSortedResults(false);

        ec_ = std::make_shared<pcl::EuclideanClusterExtraction<pcl::PointXYZI>>();
        ec_->setClusterTolerance(cfg_.cluster_tolerance);
        ec_->setMinClusterSize(cfg_.min_cluster_size);
        ec_->setMaxClusterSize(cfg_.max_cluster_size);
        ec_->setSearchMethod(tree_);
    }

    std::vector<pcl::PointIndices> extract(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const override {
        std::vector<pcl::PointIndices> cluster_indices;
        if (cloud->empty()) return cluster_indices;

        tree_->setInputCloud(cloud);
        ec_->setInputCloud(cloud);
        ec_->extract(cluster_indices);
        return cluster_indices;
    }

private:
    DetectorConfig cfg_;
    mutable std::shared_ptr<pcl::search::KdTree<pcl::PointXYZI>> tree_;
    mutable std::shared_ptr<pcl::EuclideanClusterExtraction<pcl::PointXYZI>> ec_;
};

std::unique_ptr<IClusterStrategy> create_cpu_strategy(const DetectorConfig& cfg) {
    return std::make_unique<CpuClusterStrategy>(cfg);
}

} // namespace metro
