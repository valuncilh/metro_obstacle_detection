#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <chrono>
#include <cstdint>
#include <exception>
#include <fstream>
#include <limits>
#include <memory>
#include <vector>

#include "metro_obstacle_detection/types.hpp"

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

        // Буферы живых облаков переиспользуются между кадрами: пересоздание
        // 48k-точечного облака на 10 Гц фрагментирует кучу и убивает процесс
        // по OOM на длительном прогоне.
        cloud_buf_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        downsample_buf_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        valid_cloud_buf_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

        auto qos = rclcpp::SensorDataQoS();
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/metro/points_in_gauge", qos,
            std::bind(&AnomalyDetector::on_cloud, this, std::placeholders::_1));

        pub_status_ = create_publisher<std_msgs::msg::String>("/metro/obstacles_raw", 10);
        pub_distance_ = create_publisher<std_msgs::msg::Float32>("/metro/obstacle_distance", 10);
        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/metro/markers", 10);
        pub_obstacle_points_ = create_publisher<sensor_msgs::msg::PointCloud2>("/metro/obstacle_points", qos);

        open_log();
        timer_ = create_wall_timer(std::chrono::seconds(5),
                                   std::bind(&AnomalyDetector::log_stats, this));
    }

    ~AnomalyDetector() override {
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
        declare_parameter<int>("max_points_for_clustering", cfg_.max_points_for_clustering);
        declare_parameter<float>("emergency_leaf_size", cfg_.emergency_leaf_size);
        declare_parameter<int>("max_publish_points", cfg_.max_publish_points);

        get_parameter("cluster_tolerance", cfg_.cluster_tolerance);
        get_parameter("min_cluster_size", cfg_.min_cluster_size);
        get_parameter("max_cluster_size", cfg_.max_cluster_size);
        get_parameter("min_obstacle_height", cfg_.min_obstacle_height);
        get_parameter("max_obstacle_length", cfg_.max_obstacle_length);
        get_parameter("max_obstacle_width", cfg_.max_obstacle_width);
        get_parameter("density_coefficient", cfg_.density_coefficient);
        get_parameter("max_points_for_clustering", cfg_.max_points_for_clustering);
        get_parameter("emergency_leaf_size", cfg_.emergency_leaf_size);
        get_parameter("max_publish_points", cfg_.max_publish_points);
    }

    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_buf_->clear();
        pcl::fromROSMsg(*msg, *cloud_buf_);

        if (cloud_buf_->size() < 50) {
            frames_++;
            return;
        }

        // Защита от падения на плотном облаке: выше cap — аварийное прореживание,
        // иначе экстрактор на «сплошняке» (стена в габарите) съедает память.
        const pcl::PointCloud<pcl::PointXYZI>::Ptr* cluster_input = &cloud_buf_;
        if (static_cast<int>(cloud_buf_->size()) > cfg_.max_points_for_clustering) {
            downsample_buf_->clear();
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setLeafSize(cfg_.emergency_leaf_size, cfg_.emergency_leaf_size, cfg_.emergency_leaf_size);
            vg.setInputCloud(cloud_buf_);
            vg.filter(*downsample_buf_);
            cluster_input = &downsample_buf_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "cloud %zu > cap %d, emergency downsample -> %zu",
                cloud_buf_->size(), cfg_.max_points_for_clustering, downsample_buf_->size());
        }

        try {
            auto clusters = strategy_->extract(*cluster_input);
            process_clusters(*cluster_input, clusters, msg->header);
        } catch (const std::exception& e) {
            // Сбой одного кадра не должен ронять ноду на всём прогоне.
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                "frame skipped: %s", e.what());
        }
        frames_++;
    }

    void process_clusters(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                          const std::vector<pcl::PointIndices>& clusters,
                          const std_msgs::msg::Header& header) {
        float min_dist = std::numeric_limits<float>::max();
        size_t valid_count = 0;

        valid_cloud_buf_->clear();
        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;

        for (const auto& indices : clusters) {
            if (static_cast<int>(indices.indices.size()) > cfg_.max_cluster_size) continue;

            pcl::PointCloud<pcl::PointXYZI> cluster;
            cluster.reserve(indices.indices.size());
            for (const auto idx : indices.indices) cluster.push_back(cloud->points[idx]);

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(cluster, centroid);
            const float dist = centroid[0];

            pcl::PointXYZI min_pt, max_pt;
            pcl::getMinMax3D(cluster, min_pt, max_pt);
            const float dx = max_pt.x - min_pt.x;
            const float dy = max_pt.y - min_pt.y;
            const float dz = max_pt.z - min_pt.z;

            if (dz < cfg_.min_obstacle_height ||
                dx > cfg_.max_obstacle_length ||
                dy > cfg_.max_obstacle_width) {
                continue;
            }

            const int required = std::max(cfg_.min_cluster_size,
                static_cast<int>(cfg_.density_coefficient / (dist * dist)));
            if (static_cast<int>(cluster.size()) < required) continue;

            valid_count++;
            min_dist = std::min(min_dist, dist);
            *valid_cloud_buf_ += cluster;

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
            // -1 как sentinel «нет объекта»; сравнение с FLT_MAX давало мусор в логе.
            min_dist_ = (min_dist_ < 0.0f) ? min_dist : std::min(min_dist_, min_dist);

            RCLCPP_WARN(get_logger(), "DETECTED: %zu clusters, min_dist=%.1f m", valid_count, min_dist);

            // Потолок на публикуемое облако, чтобы toROSMsg не аллоцировал гигант.
            if (static_cast<int>(valid_cloud_buf_->size()) > cfg_.max_publish_points) {
                pcl::PointCloud<pcl::PointXYZI> thinned;
                pcl::VoxelGrid<pcl::PointXYZI> vg;
                vg.setLeafSize(cfg_.emergency_leaf_size, cfg_.emergency_leaf_size, cfg_.emergency_leaf_size);
                vg.setInputCloud(valid_cloud_buf_);
                vg.filter(thinned);
                valid_cloud_buf_->swap(thinned);
            }

            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*valid_cloud_buf_, cloud_msg);
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
            std::snprintf(buf, sizeof(buf), "%s (clusters: %zu, distance: %.1f m)",
                          status.c_str(), count, distance);
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
        const auto now = std::chrono::system_clock::now();
        const auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
            now.time_since_epoch()).count();

        if (log_file_.is_open()) {
            log_file_ << timestamp << "," << frames_ << "," << detections_ << ","
                      << min_dist_ << "\n";
            log_file_.flush();
            if (++log_lines_ >= kLogRotateLines) rotate_log();
        }

        RCLCPP_INFO(get_logger(), "Stats: frames=%u, detections=%u", frames_, detections_);
        frames_ = 0;
        detections_ = 0;
        min_dist_ = -1.0f;
    }

    void open_log() {
        log_file_.open("/tmp/detection_stats.log", std::ios::app);
        if (log_file_.is_open()) {
            log_file_ << "timestamp,frames,detections,min_distance\n";
            log_lines_ = 0;
        }
    }

    // Усечение при длительной работе: без ротации файл растёт весь прогон.
    void rotate_log() {
        log_file_.close();
        log_file_.open("/tmp/detection_stats.log", std::ios::trunc);
        if (log_file_.is_open()) {
            log_file_ << "timestamp,frames,detections,min_distance\n";
            log_lines_ = 0;
        }
    }

    static constexpr uint64_t kLogRotateLines = 200000;

    metro::DetectorConfig cfg_;
    std::unique_ptr<metro::IClusterStrategy> strategy_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_buf_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsample_buf_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr valid_cloud_buf_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_distance_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_points_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::ofstream log_file_;
    uint64_t log_lines_ = 0;
    uint32_t frames_ = 0;
    uint32_t detections_ = 0;
    float min_dist_ = -1.0f;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnomalyDetector>());
    rclcpp::shutdown();
    return 0;
}
