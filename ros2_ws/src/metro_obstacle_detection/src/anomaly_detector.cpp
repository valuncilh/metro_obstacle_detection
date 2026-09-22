#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <fstream>
#include <chrono>

#include "metro_obstacle_detection/types.hpp"

/*
 * АЛГОРИТМ ОБНАРУЖЕНИЯ ПРЕПЯТСТВИЙ 
 * 
 * 1. КЛАСТЕРИЗАЦИЯ 
 *    - CPU: Euclidean Cluster Extraction (PCL) - для совместимости
 *    - GPU: CUDA-ускоренная кластеризация - для производительности
 *    - Выбор стратегии происходит на этапе компиляции (USE_CUDA flag)
 * 
 * 2. ФИЛЬТРАЦИЯ КЛАСТЕРОВ
 *    Каждый кластер проверяется по критериям:
 *    - Геометрия: отсекаем структуры тоннеля (стены, потолок, рельсы)
 *    - Плотность: требуем минимальное количество точек на дистанции
 *    - Габариты: препятствие должно быть в зоне движения поезда
 * 
 * === КЛЮЧЕВЫЕ КОЭФФИЦИЕНТЫ ===
 * 
 * cluster_tolerance (0.6 м):
 *   Максимальное расстояние между точками в одном кластере.
 *   Выбрано на основе типичной плотности облака точек лидара:
 *   - Ближняя зона (5-30 м): ~1000 точек/м²
 *   - Дальняя зона (100-300 м): ~10 точек/м²
 *   Значение 0.6 м объединяет точки объекта, но разделяет близкие объекты.
 * 
 * min_obstacle_height (0.15 м):
 *   Минимальная высота препятствия над уровнем рельс.
 *   Лидар установлен на высоте платформы (~1.1 м), поэтому:
 *   - Точки на Z < 0.15 м это рельсы и шпалы (игнорируем)
 *   - Точки на Z > 0.15 м это потенциальные препятствия
 * 
 * max_obstacle_width (2.7 м):
 *   Максимальная ширина препятствия.
 *   Ширина поезда "Москва-2024" = 2.7 м, поэтому:
 *   - Объекты шире 2.7 м это стены тоннеля (игнорируем)
 *   - Объекты уже это препятствия на пути
 * 
 * max_obstacle_length (5.0 м):
 *   Максимальная длина препятствия вдоль оси движения.
 *   Отсекает длинные структуры (кабели, трубы).
 * 
 * density_coefficient (900.0):
 *   Коэффициент для адаптивного порога плотности.
 *   Требуемое количество точек = density_coefficient / distance²
 *   Физический смысл: плотность облака падает как 1/r²
 *   При distance=10 м требуется 9 точек, при distance=100 м требуется 1 точка.
 *   Это компенсирует разреженность облака на больших дистанциях.
 */

namespace metro {
    std::unique_ptr<IClusterStrategy> create_cpu_strategy(const DetectorConfig& cfg);
#ifdef USE_CUDA
    std::unique_ptr<IClusterStrategy> create_gpu_strategy(const DetectorConfig& cfg);
#endif
}

class AnomalyDetector : public rclcpp::Node {
public:
    AnomalyDetector() : Node("anomaly_detector") {
        load_config();

#ifdef USE_CUDA
        strategy_ = metro::create_gpu_strategy(cfg_);
        RCLCPP_INFO(get_logger(), "Started with GPU acceleration");
#else
        strategy_ = metro::create_cpu_strategy(cfg_);
        RCLCPP_INFO(get_logger(), "Started in CPU mode");
#endif

        auto qos = rclcpp::SensorDataQoS();
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/metro/points_in_gauge", qos,
            std::bind(&AnomalyDetector::on_cloud, this, std::placeholders::_1));

        pub_status_ = create_publisher<std_msgs::msg::String>("/metro/obstacles_raw", 10);
        pub_distance_ = create_publisher<std_msgs::msg::Float32>("/metro/obstacle_distance", 10);
        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/metro/markers", 10);
        pub_obstacle_points_ = create_publisher<sensor_msgs::msg::PointCloud2>("/metro/obstacle_points", qos);

        log_file_.open("/tmp/detection_stats.log", std::ios::app);
        log_file_ << "timestamp,frames,detections,min_distance\n";

        timer_ = create_wall_timer(std::chrono::seconds(5),
                                   std::bind(&AnomalyDetector::log_stats, this));
    }

    ~AnomalyDetector() {
        if (log_file_.is_open()) log_file_.close();
    }

private:
    void load_config() {
        declare_parameter<float>("cluster_tolerance", cfg_.cluster_tolerance);
        declare_parameter<int>("min_cluster_size", cfg_.min_cluster_size);
        declare_parameter<int>("max_cluster_size", cfg_.max_cluster_size);
        declare_parameter<float>("min_obstacle_height", cfg_.min_obstacle_height);
        declare_parameter<float>("max_obstacle_length", cfg_.max_obstacle_length);
        declare_parameter<float>("max_obstacle_width", cfg_.max_obstacle_width);
        declare_parameter<float>("density_coefficient", cfg_.density_coefficient);

        get_parameter("cluster_tolerance", cfg_.cluster_tolerance);
        get_parameter("min_cluster_size", cfg_.min_cluster_size);
        get_parameter("max_cluster_size", cfg_.max_cluster_size);
        get_parameter("min_obstacle_height", cfg_.min_obstacle_height);
        get_parameter("max_obstacle_length", cfg_.max_obstacle_length);
        get_parameter("max_obstacle_width", cfg_.max_obstacle_width);
        get_parameter("density_coefficient", cfg_.density_coefficient);
    }

    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->size() < 50) {
            frames_++;
            return;
        }

        auto clusters = strategy_->extract(cloud);
        process_clusters(cloud, clusters, msg->header);
        frames_++;
    }

    /*
     * === ФИЛЬТРАЦИЯ КЛАСТЕРОВ ===
     * 
     * Для каждого кластера:
     * 1. Вычисляем геометрические характеристики 
     * 2. Проверяем критерии:
     *    - dz < min_obstacle_height → структура (рельсы)
     *    - dx > max_obstacle_length → длинная структура (кабели)
     *    - dy > max_obstacle_width → стена тоннеля
     *    - points < required → шум или слишком разреженный объект
     * 3. Если все проверки пройдены → препятствие
     */
    void process_clusters(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                          const std::vector<pcl::PointIndices>& clusters,
                          const std_msgs::msg::Header& header) {
        float min_dist = std::numeric_limits<float>::max();
        size_t valid_count = 0;
        
        auto valid_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;

        for (const auto& indices : clusters) {
            auto cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pcl::copyPointCloud(*cloud, indices, *cluster);

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);
            float dist = centroid[0];

            pcl::PointXYZI min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
            
            float dx = max_pt.x - min_pt.x;
            float dy = max_pt.y - min_pt.y;
            float dz = max_pt.z - min_pt.z;

            // Геометрическая фильтрация
            if (dz < cfg_.min_obstacle_height || 
                dx > cfg_.max_obstacle_length || 
                dy > cfg_.max_obstacle_width) {
                continue;
            }

            // Адаптивный порог плотности
            int required = std::max(cfg_.min_cluster_size, 
                                    static_cast<int>(cfg_.density_coefficient / (dist * dist)));
            if (static_cast<int>(cluster->size()) < required) {
                continue;
            }

            valid_count++;
            min_dist = std::min(min_dist, dist);
            *valid_cloud += *cluster;

            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "obstacles";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = centroid[0];
            marker.pose.position.y = centroid[1];
            marker.pose.position.z = centroid[2];
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
            marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f; marker.color.a = 0.8f;
            marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            markers.markers.push_back(marker);
        }

        if (valid_count > 0) {
            detections_ += valid_count;
            min_dist_ = std::min(min_dist_, min_dist);
            RCLCPP_WARN(get_logger(), "DETECTED: %zu clusters, min_dist=%.1f m", valid_count, min_dist);
            
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*valid_cloud, cloud_msg);
            cloud_msg.header = header;
            pub_obstacle_points_->publish(cloud_msg);
        }

        pub_markers_->publish(markers);
        publish_result(valid_count > 0 ? "obstacle_detected" : "no_obstacle", valid_count, min_dist);
    }

    void publish_result(const std::string& status, size_t count, float distance) {
        std_msgs::msg::String status_msg;
        if (distance >= 0) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "%s (clusters: %zu, distance: %.1f m)", status.c_str(), count, distance);
            status_msg.data = buf;
        } else {
            status_msg.data = status;
        }
        pub_status_->publish(status_msg);

        if (distance >= 0) {
            std_msgs::msg::Float32 dist_msg;
            dist_msg.data = distance;
            pub_distance_->publish(dist_msg);
        }
    }

    void log_stats() {
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        
        if (log_file_.is_open()) {
            log_file_ << timestamp << "," << frames_ << "," << detections_ << "," 
                      << (min_dist_ > 0 ? min_dist_ : -1.0) << "\n";
            log_file_.flush();
        }

        RCLCPP_INFO(get_logger(), "Stats: frames=%u, detections=%u", frames_, detections_);
        
        frames_ = 0;
        detections_ = 0;
        min_dist_ = std::numeric_limits<float>::max();
    }

    metro::DetectorConfig cfg_;
    std::unique_ptr<metro::IClusterStrategy> strategy_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_distance_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_points_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::ofstream log_file_;
    uint32_t frames_ = 0;
    uint32_t detections_ = 0;
    float min_dist_ = std::numeric_limits<float>::max();
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnomalyDetector>());
    rclcpp::shutdown();
    return 0;
}
