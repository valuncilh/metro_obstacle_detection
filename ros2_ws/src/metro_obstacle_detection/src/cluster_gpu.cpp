#include "metro_obstacle_detection/types.hpp"

#include <map>
#include <vector>
#include <memory>

namespace metro {
namespace gpu {
struct ClusterBuffers;
ClusterBuffers* cluster_buffers_create();
void cluster_buffers_destroy(ClusterBuffers*);
int cluster_run(ClusterBuffers*, const float*, int, float, int*);
} // namespace gpu
} // namespace metro

namespace metro {

class GpuClusterStrategy : public IClusterStrategy {
public:
    explicit GpuClusterStrategy(const DetectorConfig& cfg) : cfg_(cfg) {
        bufs_ = gpu::cluster_buffers_create();
    }

    // CUDA-ресурс не RAII-обёрнут, поэтому деструктор явный — единственное
    // место в проекте, где Rule of Zero сознательно нарушен.
    ~GpuClusterStrategy() override { gpu::cluster_buffers_destroy(bufs_); }

    std::vector<pcl::PointIndices> extract(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const override {
        std::vector<pcl::PointIndices> cluster_indices;
        const int n = static_cast<int>(cloud->size());
        if (n == 0 || !bufs_) return cluster_indices;

        // resize держит capacity между кадрами — не пересоздаём векторы на 10 Гц.
        h_points_.resize((size_t)n * 4);
        for (int i = 0; i < n; ++i) {
            h_points_[(size_t)i*4]   = cloud->points[i].x;
            h_points_[(size_t)i*4+1] = cloud->points[i].y;
            h_points_[(size_t)i*4+2] = cloud->points[i].z;
            h_points_[(size_t)i*4+3] = cloud->points[i].intensity;
        }
        h_labels_.resize(n);

        const float tol_sq = cfg_.cluster_tolerance * cfg_.cluster_tolerance;
        if (gpu::cluster_run(bufs_, h_points_.data(), n, tol_sq, h_labels_.data()) != 0) {
            return cluster_indices;  // сбой GPU-кадра: пустой результат, нода жива
        }

        label_map_.clear();
        for (int i = 0; i < n; ++i) label_map_[h_labels_[i]].indices.push_back(i);
        for (auto& [label, indices] : label_map_) {
            if (static_cast<int>(indices.indices.size()) >= cfg_.min_cluster_size) {
                cluster_indices.push_back(std::move(indices));
            }
        }
        return cluster_indices;
    }

private:
    DetectorConfig cfg_;
    gpu::ClusterBuffers* bufs_ = nullptr;
    mutable std::vector<float> h_points_;
    mutable std::vector<int> h_labels_;
    mutable std::map<int, pcl::PointIndices> label_map_;
};

std::unique_ptr<IClusterStrategy> create_gpu_strategy(const DetectorConfig& cfg) {
    return std::make_unique<GpuClusterStrategy>(cfg);
}

} // namespace metro
