#include <cuda_runtime.h>

// Только float*/int*/bool*: nvcc не должен видеть PCL/Eigen, иначе
// -Wpedantic на анонимных структурах PCL и #line-директивы nvcc дают
// лавину варнингов и нестабильную/убитую по OOM сборку при USE_CUDA=ON.

namespace metro {
namespace gpu {

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
    int parent = labels[l];
    int m = (l < parent) ? l : parent;   // без min(): не зависеть от перегрузок device-рантайма
    if (m != l) {
        atomicMin(&labels[i], m);
        *changed = true;
    }
}

struct ClusterBuffers {
    float* d_points = nullptr;
    int*   d_labels = nullptr;
    bool*  d_changed = nullptr;
    int    capacity = 0;
};

ClusterBuffers* cluster_buffers_create() { return new ClusterBuffers(); }

void cluster_buffers_destroy(ClusterBuffers* b) {
    if (!b) return;
    if (b->d_points)  cudaFree(b->d_points);
    if (b->d_labels)  cudaFree(b->d_labels);
    if (b->d_changed) cudaFree(b->d_changed);
    delete b;
}

// 0 = ok, иначе cudaError_t. Вызывающий решает судьбу кадра: молчаливый
// пропуск лучше OOM-kill'а процесса на середине прогона.
int cluster_run(ClusterBuffers* b, const float* h_points, int n,
                float tol_sq, int* h_labels_out) {
    if (n <= 0) return 0;

    // Буферы растят capacity один раз и переиспользуются: cudaMalloc/Free
    // каждый кадр фрагментируют память GPU и роняют длительный прогон.
    if (n > b->capacity) {
        if (b->d_points) cudaFree(b->d_points);
        if (b->d_labels) cudaFree(b->d_labels);
        const int new_cap = (n < 20000) ? 20000 : n * 2;  // запас, чтобы не реаллоцировать каждый кадр
        cudaError_t e = cudaMalloc(&b->d_points, (size_t)new_cap * 4 * sizeof(float));
        if (e != cudaSuccess) return (int)e;
        e = cudaMalloc(&b->d_labels, (size_t)new_cap * sizeof(int));
        if (e != cudaSuccess) return (int)e;
        if (!b->d_changed) {
            e = cudaMalloc(&b->d_changed, sizeof(bool));
            if (e != cudaSuccess) return (int)e;
        }
        b->capacity = new_cap;
    }

    cudaError_t e = cudaMemcpy(b->d_points, h_points,
                               (size_t)n * 4 * sizeof(float), cudaMemcpyHostToDevice);
    if (e != cudaSuccess) return (int)e;

    const int threads = 256;
    const int blocks  = (n + threads - 1) / threads;

    initLabelsKernel<<<blocks, threads>>>(b->d_labels, n);
    linkKernel<<<blocks, threads>>>(b->d_points, b->d_labels, n, tol_sq);
    e = cudaGetLastError();
    if (e != cudaSuccess) return (int)e;

    // 64 прохода покрывают цепочки связности; после CropBox+voxel точек мало,
    // схода хватает с большим запасом.
    bool changed = true;
    for (int iter = 0; iter < 64 && changed; ++iter) {
        changed = false;
        cudaMemcpy(b->d_changed, &changed, sizeof(bool), cudaMemcpyHostToDevice);
        resolveKernel<<<blocks, threads>>>(b->d_labels, n, b->d_changed);
        cudaDeviceSynchronize();
        cudaMemcpy(&changed, b->d_changed, sizeof(bool), cudaMemcpyDeviceToHost);
    }

    e = cudaMemcpy(h_labels_out, b->d_labels, (size_t)n * sizeof(int), cudaMemcpyDeviceToHost);
    return (int)e;
}

} // namespace gpu
} // namespace metro
