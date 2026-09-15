#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

class AnomalyDetector : public rclcpp::Node {
public:
    AnomalyDetector() : Node("anomaly_detector") {
        auto qos = rclcpp::SensorDataQoS();

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/metro/points_in_gauge", qos,
            std::bind(&AnomalyDetector::cloudCallback, this, std::placeholders::_1));

        pub_obstacles_ = create_publisher<std_msgs::msg::String>(
            "/metro/obstacles_raw", 10);

        RCLCPP_INFO(get_logger(), "Anomaly detector node started");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            publishResult("no_obstacle", 0);
            return;
        }

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(15);
        ec.setMaxClusterSize(50000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        if (cluster_indices.empty()) {
            publishResult("no_obstacle", 0);
        } else {
            publishResult("obstacle_detected", cluster_indices.size());
        }
    }

    void publishResult(const std::string& status, size_t cluster_count) {
        std_msgs::msg::String msg;
        msg.data = status + " (clusters: " + std::to_string(cluster_count) + ")";
        pub_obstacles_->publish(msg);
        RCLCPP_INFO(get_logger(), "Detection: %s", msg.data.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_obstacles_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnomalyDetector>());
    rclcpp::shutdown();
    return 0;
}
