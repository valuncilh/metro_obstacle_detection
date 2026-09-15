#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

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
        pcl::PassThrough<pcl::PointXYZI> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(5.0, 300.0);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_x(new pcl::PointCloud<pcl::PointXYZI>);
        pass_x.filter(*filtered_x);

        // Прореживание
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(filtered_x);
        voxel.setLeafSize(0.1f, 0.1f, 0.1f);
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
        voxel.filter(*downsampled);

        pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);

        // Защита: если точек слишком мало, пропускаем RANSAC
        if (downsampled->size() >= kMinPointsForRansac) {
            pcl::SACSegmentation<pcl::PointXYZI> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.15);
            seg.setMaxIterations(100);
            seg.setInputCloud(downsampled);
            seg.segment(*inliers, *coefficients);

            if (!inliers->indices.empty()) {
                pcl::ExtractIndices<pcl::PointXYZI> extract;
                extract.setInputCloud(downsampled);
                extract.setIndices(inliers);
                extract.setNegative(true);
                extract.filter(*result);
            } else {
                // Плоскость не найдена — оставляем как есть
                *result = *downsampled;
            }
        } else {
            // Слишком мало точек — пропускаем удаление земли
            *result = *downsampled;
        }

        // Диагностическая гистограмма
        frame_counter_++;
        if (frame_counter_ % 30 == 0) {
            logDistanceHistogram(*result);
        }

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*result, output_msg);
        output_msg.header = msg->header;
        pub_->publish(output_msg);
    }

    void logDistanceHistogram(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
        long bins[5] = {0, 0, 0, 0, 0};
        for (const auto& p : cloud) {
            if (p.x < 25.0f)       bins[0]++;
            else if (p.x < 50.0f)  bins[1]++;
            else if (p.x < 100.0f) bins[2]++;
            else if (p.x < 200.0f) bins[3]++;
            else                   bins[4]++;
        }
        RCLCPP_INFO(get_logger(),
            "Distance histogram: 5-25m:%ld 25-50m:%ld 50-100m:%ld 100-200m:%ld 200-300m:%ld (total:%zu)",
            bins[0], bins[1], bins[2], bins[3], bins[4], cloud.size());
    }

    static constexpr size_t kMinPointsForRansac = 100;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    uint32_t frame_counter_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Preprocessor>());
    rclcpp::shutdown();
    return 0;
}
