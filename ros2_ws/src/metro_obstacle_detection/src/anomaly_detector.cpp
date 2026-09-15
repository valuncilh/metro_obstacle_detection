#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>

#include <algorithm>
#include <cmath>
#include <limits>

class AnomalyDetector : public rclcpp::Node {
public:
    AnomalyDetector() : Node("anomaly_detector") {
        auto qos = rclcpp::SensorDataQoS();

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/metro/points_in_gauge", qos,
            std::bind(&AnomalyDetector::cloudCallback, this, std::placeholders::_1));

        pub_status_ = create_publisher<std_msgs::msg::String>("/metro/obstacles_raw", 10);
        pub_distance_ = create_publisher<std_msgs::msg::Float32>("/metro/obstacle_distance", 10);
        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/metro/markers", 10);

        RCLCPP_INFO(get_logger(), "Anomaly detector node started");
    }

private:
    // Требуемое число точек в кластере в зависимости от дистанции.
    // Плотность облака падает как 1/D^2, поэтому порог тоже должен падать.
    // k = 900 подобрано по объекту из doubleT_obstacle (130 точек на 6.6 м).
    static int requiredPoints(float distance) {
        return std::max(kMinClusterPoints,
                        static_cast<int>(kDensityCoeff / (distance * distance)));
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->size() < 50) {
            publishResult("no_obstacle", 0, -1.0f, msg->header);
            return;
        }

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud);

        // Извлекаем все кластеры с низким порогом — отбраковка будет ниже, по дистанции.
        // Допуск 0.6 м: на 200 м соседние точки разъезжаются до ~0.52 м,
        // при 0.5 м дальний объект распадался бы на фрагменты.
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.6);
        ec.setMinClusterSize(kMinClusterPoints);
        ec.setMaxClusterSize(100000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        float min_distance = std::numeric_limits<float>::max();
        size_t kept_count = 0;
        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;

        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
            cluster->reserve(indices.indices.size());
            for (const auto idx : indices.indices) {
                cluster->push_back(cloud->points[idx]);
            }

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);
            const float dist = centroid[0];

            // Адаптивная отбраковка: сколько точек должен иметь объект на этой дистанции
            const int required = requiredPoints(dist);
            const int actual = static_cast<int>(cluster->size());

            if (actual < required) {
                RCLCPP_DEBUG(get_logger(),
                    "Rejected cluster: dist=%.1f m, points=%d < required=%d",
                    dist, actual, required);
                continue;
            }

            kept_count++;
            min_distance = std::min(min_distance, dist);

            RCLCPP_INFO(get_logger(),
                "  Cluster: pos=(%.1f, %.1f, %.1f), points=%d (required>=%d), dist=%.1f m",
                centroid[0], centroid[1], centroid[2], actual, required, dist);

            visualization_msgs::msg::Marker marker;
            marker.header = msg->header;
            marker.ns = "obstacles";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = centroid[0];
            marker.pose.position.y = centroid[1];
            marker.pose.position.z = centroid[2];
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.8f;
            marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            markers.markers.push_back(marker);
        }

        pub_markers_->publish(markers);

        if (kept_count == 0) {
            publishResult("no_obstacle", 0, -1.0f, msg->header);
        } else {
            publishResult("obstacle_detected", kept_count, min_distance, msg->header);
        }
    }

    void publishResult(const std::string& status, size_t count, float distance,
                       const std_msgs::msg::Header& /*header*/) {
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

        RCLCPP_INFO(get_logger(), "Detection: %s", status_msg.data.c_str());
    }

    static constexpr int kMinClusterPoints = 6;
    static constexpr float kDensityCoeff = 900.0f;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_distance_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnomalyDetector>());
    rclcpp::shutdown();
    return 0;
}
