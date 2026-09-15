#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

class Preprocessor : public rclcpp::Node {
public:
    Preprocessor() : Node("preprocessor") {
        auto qos = rclcpp::SensorDataQoS();

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", qos,
            std::bind(&Preprocessor::cloudCallback, this, std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/metro/points_preprocessed", qos);

        RCLCPP_INFO(get_logger(), "Preprocessor node started");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        // Фильтр по дальности
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(1.0, 300.0);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pass.filter(*filtered);

        // Voxel grid прореживание
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(filtered);
        voxel.setLeafSize(0.15f, 0.15f, 0.15f);
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
        voxel.filter(*downsampled);

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*downsampled, output_msg);
        output_msg.header = msg->header;
        pub_->publish(output_msg);

        RCLCPP_DEBUG(get_logger(), "Preprocessed: %zu -> %zu points",
                     cloud->size(), downsampled->size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Preprocessor>());
    rclcpp::shutdown();
    return 0;
}
