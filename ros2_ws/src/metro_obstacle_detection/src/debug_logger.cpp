#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

class DebugLogger : public rclcpp::Node {
public:
    DebugLogger() : Node("debug_logger") {
        auto qos = rclcpp::SensorDataQoS();

        sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "[LIDAR] frame_id=%s, points=%u",
                    msg->header.frame_id.c_str(), msg->width * msg->height);
            });

        sub_preproc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/metro/points_preprocessed", qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "[PREPROC] points=%u", msg->width * msg->height);
            });

        sub_gauge_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/metro/points_in_gauge", qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "[GAUGE] points=%u", msg->width * msg->height);
            });

        sub_obstacles_ = create_subscription<std_msgs::msg::String>(
            "/metro/obstacles_raw", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "[OBSTACLES] %s", msg->data.c_str());
            });

        sub_safety_ = create_subscription<std_msgs::msg::String>(
            "/metro/safety_status", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "[SAFETY] %s", msg->data.c_str());
            });

        RCLCPP_INFO(get_logger(), "Debug logger started — monitoring all topics");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_preproc_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_gauge_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_obstacles_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_safety_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugLogger>());
    rclcpp::shutdown();
    return 0;
}
