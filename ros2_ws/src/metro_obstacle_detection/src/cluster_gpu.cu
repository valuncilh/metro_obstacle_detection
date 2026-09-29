#include "metro_obstacle_detection/types.hpp"
#include <cuda_runtime.h>
#include <map>

// Связные компоненты: linkKernel строит рёбра (atomicMin по обоим концам),
// resolveKernel доводит метку до минимума компоненты pointer-jumping'ом.
// Один проход «точка -> ближайший сосед» без этого даёт пары вместо цепочек,
// поэтому объект дробился на 4-6 кластеров.
// O(n^2) скан приемлем до ~10^4 точек после фильтрации; выше — воксельный поиск.

namespace metro {

__global__ void initLabelsKernel(int* labels, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) labels[i] = i;
}

__global__ void linkKernel(const float* points, int* labels, int n, float tol_sq) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    float px = points[i*4], py = points[i*4+1], pz = points[i*4+2];
    for (int j = 0; j < i; ++j) {
        float dx = px - points[j*4];
        float dy = py - points[j*4+1];
        float dz = pz - points[j*4+2];
        if (dx*dx + dy*dy + dz*dz <= tol_sq) {
            atomicMin(&labels[i], j);
            atomicMin(&labels[j], i);
        }
    }
}

__global__ void resolveKernel(int* labels, int n, bool* changed) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    int l = labels[i];
    int m = min(l, labels[l]);
    if (m != l) {
        atomicMin(&labels[i], m);
        *changed = true;
    }
}

class GpuClusterStrategy : public IClusterStrategy {
public:
    explicit GpuClusterStrategy(const DetectorConfig& cfg) : cfg_(cfg) {}

    std::vector<pcl::PointIndices> extract(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const override {
        std::vector<pcl::PointIndices> cluster_indices;
        const int n = static_cast<int>(cloud->size());
        if (n == 0) return cluster_indices;

        std::vector<float> h_points(n * 4);
        for (int i = 0; i < n; ++i) {
            h_points[i*4]   = cloud->points[i].x;
            h_points[i*4+1] = cloud->points[i].y;
            h_points[i*4+2] = cloud->points[i].z;
            h_points[i*4+3] = cloud->points[i].intensity;
        }

        float* d_points = nullptr;
        int* d_labels = nullptr;
        bool* d_changed = nullptr;
        cudaMalloc(&d_points, n * 4 * sizeof(float));
        cudaMalloc(&d_labels, n * sizeof(int));
        cudaMalloc(&d_changed, sizeof(bool));
        cudaMemcpy(d_points, h_points.data(), n * 4 * sizeof(float), cudaMemcpyHostToDevice);

        const int threads = 256;
        const int blocks = (n + threads - 1) / threads;
        const float tol_sq = cfg_.cluster_tolerance * cfg_.cluster_tolerance;

        initLabelsKernel<<<blocks, threads>>>(d_labels, n);
        linkKernel<<<blocks, threads>>>(d_points, d_labels, n, tol_sq);

        // 64 итерации покрывают цепочки длиной до 64; после CropBox точек мало,
        // схода хватает с запасом.
        bool changed = true;
        for (int iter = 0; iter < 64 && changed; ++iter) {
            changed = false;
            cudaMemcpy(d_changed, &changed, sizeof(bool), cudaMemcpyHostToDevice);
            resolveKernel<<<blocks, threads>>>(d_labels, n, d_changed);
            cudaDeviceSynchronize();
            cudaMemcpy(&changed, d_changed, sizeof(bool), cudaMemcpyDeviceToHost);
        }

        std::vector<int> h_labels(n);
        cudaMemcpy(h_labels.data(), d_labels, n * sizeof(int), cudaMemcpyDeviceToHost);
        cudaFree(d_points);
        cudaFree(d_labels);
        cudaFree(d_changed);

        std::map<int, pcl::PointIndices> label_map;
        for (int i = 0; i < n; ++i) label_map[h_labels[i]].indices.push_back(i);
        for (auto& [label, indices] : label_map) {
            if (static_cast<int>(indices.indices.size()) >= cfg_.min_cluster_size) {
                cluster_indices.push_back(std::move(indices));
            }
        }
        return cluster_indices;
    }

private:
    DetectorConfig cfg_;
};

std::unique_ptr<IClusterStrategy> create_gpu_strategy(const DetectorConfig& cfg) {
    return std::make_unique<GpuClusterStrategy>(cfg);
}

} // namespace metro
