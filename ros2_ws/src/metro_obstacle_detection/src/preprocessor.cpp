#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

/*
 * ПРЕДОБРАБОТКА ОБЛАКА ТОЧЕК 
 * 
 * Этот узел выполняет трехэтапную фильтрацию входного облака:
 * 
 * 1. PASS THROUGH (X-axis)
 *    Отсекаем точки ближе 2 м и дальше 300 м.
 *    - Ближе 2 м: точки корпуса поезда (не интересуют)
 *    - Дальше 300 м: за пределами рабочей зоны лидара
 * 
 * 2. VOXEL GRID DOWNSAMPLING (0.15 м)
 *    Уменьшаем плотность облака для ускорения обработки.
 *    Выбор 0.15 м — компромисс между скоростью и точностью:
 *    - 0.10 м: слишком медленно для real-time
 *    - 0.20 м: теряет мелкие объекты на дальних дистанциях
 *    - 0.15 м: оптимально для объектов > 40 см
 * 
 * 3. RANSAC PLANE SEGMENTATION
 *    Удаляем плоскость пола/рельсов.
 *    - Distance threshold (0.15 м): точки в пределах 15 см от плоскости
 *    - Max iterations (100): достаточно для сходимости на плоских поверхностях
 *    После RANSAC применяем z_safety фильтр для удаления остатков.
 * 
 * === ГЕОМЕТРИЯ УСТАНОВКИ ===
 * 
 * Лидар установлен на высоте платформы метро (~1.1 м от уровня рельс).
 * В системе координат лидара:
 * - X: вперед по движению поезда
 * - Y: влево (положительное) / вправо (отрицательное)
 * - Z: вверх от лидара
 * 
 * Поэтому уровень рельс находится на Z ≈ -1.1 м.
 */

class Preprocessor : public rclcpp::Node {
public:
    Preprocessor() : Node("preprocessor") {
        declare_parameter<float>("min_z_safety", -1.3f);
        declare_parameter<float>("max_z_safety", 1.3f);

        get_parameter("min_z_safety", min_z_safety_);
        get_parameter("max_z_safety", max_z_safety_);

        auto qos = rclcpp::SensorDataQoS();
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", qos,
            std::bind(&Preprocessor::cloudCallback, this, std::placeholders::_1));
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/metro/points_preprocessed", qos);

        RCLCPP_INFO(get_logger(), "Preprocessor started, z_safety[%.2f,%.1f]",
                    min_z_safety_, max_z_safety_);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*msg, *cloud);

        // 1. Фильтр по дальности
        pcl::PassThrough<pcl::PointXYZI> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(2.0, 300.0);
        auto filtered_x = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pass_x.filter(*filtered_x);

        // 2. Прореживание (ускорение в ~3 раза)
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(filtered_x);
        voxel.setLeafSize(0.15f, 0.15f, 0.15f);
        auto downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        voxel.filter(*downsampled);

        auto result = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        // 3. Удаление плоскости пола
        if (downsampled->size() >= 100) {
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
                *result = *downsampled;
            }
        } else {
            *result = *downsampled;
        }

        // Защитный фильтр по Z: удаляем остатки пола и шум
        pcl::PassThrough<pcl::PointXYZI> z_safety;
        z_safety.setInputCloud(result);
        z_safety.setFilterFieldName("z");
        z_safety.setFilterLimits(min_z_safety_, max_z_safety_);
        auto final_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        z_safety.filter(*final_cloud);

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*final_cloud, output_msg);
        output_msg.header = msg->header;
        pub_->publish(output_msg);
    }

    float min_z_safety_ = -1.3f;
    float max_z_safety_ = 1.3f;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Preprocessor>());
    rclcpp::shutdown();
    return 0;
}
