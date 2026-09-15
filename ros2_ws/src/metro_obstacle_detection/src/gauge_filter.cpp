#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

class GaugeFilter : public rclcpp::Node {
public:
    GaugeFilter() : Node("gauge_filter") {
        auto qos = rclcpp::SensorDataQoS();

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/metro/points_preprocessed", qos,
            std::bind(&GaugeFilter::cloudCallback, this, std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/metro/points_in_gauge", qos);

        RCLCPP_INFO(get_logger(), "Gauge filter node started");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::CropBox<pcl::PointXYZI> crop;
        crop.setInputCloud(cloud);
        crop.setMin(Eigen::Vector4f(-2.0f, -1.5f, -0.5f, 1.0f));
        crop.setMax(Eigen::Vector4f(300.0f, 1.5f, 4.0f, 1.0f));

        pcl::PointCloud<pcl::PointXYZI>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZI>);
        crop.filter(*cropped);

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cropped, output_msg);
        output_msg.header = msg->header;
        pub_->publish(output_msg);

        RCLCPP_DEBUG(get_logger(), "Gauge filter: %zu -> %zu points",
                     cloud->size(), cropped->size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaugeFilter>());
    rclcpp::shutdown();
    return 0;
}
