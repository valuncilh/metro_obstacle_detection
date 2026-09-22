#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

/*
 *  ФИЛЬТР ГАБАРИТА 
 * 
 * Отсекает точки вне зоны движения поезда.
 * 
 * Геометрические параметры:
 * - Поезд "Москва-2024": ширина 2.7 м, высота 3.72 м
 * - Колея: 1520 мм (расстояние между рельсами)
 * - Лидар: установлен на высоте платформы (~1.1 м от рельс)
 * 
 * Зона поиска (CropBox):
 * - X: [2.0, 300.0] м — от 2 м впереди до 300 м (предел лидара)
 * - Y: [-1.5, 1.5] м — ±1.5 м от оси (колея 1.52 м + запас 0.5 м)
 * - Z: [-1.1, 3.0] м — от уровня рельс до высоты тоннеля
 * 
 * Почему именно эти значения:
 * - min_x = 2.0 м: ближе находится корпус поезда
 * - max_x = 300.0 м: максимальная дальность лидара
 * - ±1.5 м по Y: покрывает колею с запасом для кривых участков
 * - Z от -1.1 м: учитывает, что лидар выше рельс на 1.1 м
 */

class GaugeFilter : public rclcpp::Node {
public:
    GaugeFilter() : Node("gauge_filter") {
        declare_parameter<float>("min_x", 2.0f);
        declare_parameter<float>("max_x", 300.0f);
        declare_parameter<float>("min_y", -1.5f);
        declare_parameter<float>("max_y", 1.5f);
        declare_parameter<float>("min_z", -1.1f);
        declare_parameter<float>("max_z", 3.0f);

        float min_x, max_x, min_y, max_y, min_z, max_z;
        get_parameter("min_x", min_x);
        get_parameter("max_x", max_x);
        get_parameter("min_y", min_y);
        get_parameter("max_y", max_y);
        get_parameter("min_z", min_z);
        get_parameter("max_z", max_z);

        crop_box_.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0f));
        crop_box_.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0f));

        auto qos = rclcpp::SensorDataQoS();
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/metro/points_preprocessed", qos,
            std::bind(&GaugeFilter::cloudCallback, this, std::placeholders::_1));
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/metro/points_in_gauge", qos);

        RCLCPP_INFO(get_logger(), "Gauge filter: x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]",
                    min_x, max_x, min_y, max_y, min_z, max_z);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*msg, *cloud);

        crop_box_.setInputCloud(cloud);
        auto cropped = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        crop_box_.filter(*cropped);

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cropped, output_msg);
        output_msg.header = msg->header;
        pub_->publish(output_msg);
    }

    pcl::CropBox<pcl::PointXYZI> crop_box_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaugeFilter>());
    rclcpp::shutdown();
    return 0;
}
