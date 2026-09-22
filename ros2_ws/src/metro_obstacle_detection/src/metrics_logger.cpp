#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <map>
#include <mutex>

using sensor_msgs::msg::PointCloud2;
using std_msgs::msg::String;
using visualization_msgs::msg::MarkerArray;

// Замеряет FPS по стадиям пайплайна и задержки обработки.
// Задержки считаются по совпадению header.stamp: все ноды пробрасывают
// stamp входного кадра в выходной, поэтому можно сопоставить время
// получения одного и того же кадра разными нодами.
class MetricsLogger : public rclcpp::Node {
public:
    MetricsLogger() : Node("metrics_logger") {
        auto qos = rclcpp::SensorDataQoS();

        sub_lidar_ = create_subscription<PointCloud2>(
            "/lidar_points", qos,
            [this](const PointCloud2::SharedPtr m) { onInput(m); });
        sub_pre_ = create_subscription<PointCloud2>(
            "/metro/points_preprocessed", qos,
            [this](const PointCloud2::SharedPtr m) { onPre(m); });
        sub_gauge_ = create_subscription<PointCloud2>(
            "/metro/points_in_gauge", qos,
            [this](const PointCloud2::SharedPtr m) { onGauge(m); });
        sub_markers_ = create_subscription<MarkerArray>(
            "/metro/markers", 10,
            [this](const MarkerArray::SharedPtr m) { onMarkers(m); });
        sub_status_ = create_subscription<String>(
            "/metro/obstacles_raw", 10,
            [this](const String::SharedPtr) {
                std::lock_guard<std::mutex> lk(mu_);
                count_det_++;
            });

        timer_ = create_wall_timer(std::chrono::seconds(1),
                                   std::bind(&MetricsLogger::report, this));
        last_report_ = this->now();
        RCLCPP_INFO(get_logger(), "[metrics] logger started");
    }

private:
    static int64_t stampKey(const std_msgs::msg::Header& h) {
        return rclcpp::Time(h.stamp).nanoseconds();
    }

    static void prune(std::map<int64_t, rclcpp::Time>& m) {
        while (m.size() > kMaxEntries) {
            m.erase(m.begin());
        }
    }

    void onInput(const PointCloud2::SharedPtr& m) {
        std::lock_guard<std::mutex> lk(mu_);
        recv_lidar_[stampKey(m->header)] = this->now();
        count_lidar_++;
        last_points_in_ = m->width * m->height;
        prune(recv_lidar_);
    }

    void onPre(const PointCloud2::SharedPtr& m) {
        const auto now = this->now();
        const int64_t key = stampKey(m->header);
        std::lock_guard<std::mutex> lk(mu_);
        recv_pre_[key] = now;
        count_pre_++;
        const auto it = recv_lidar_.find(key);
        if (it != recv_lidar_.end()) {
            lat_pre_sum_ += (now - it->second).seconds();
            lat_pre_n_++;
        }
        prune(recv_pre_);
    }

    void onGauge(const PointCloud2::SharedPtr& m) {
        const auto now = this->now();
        const int64_t key = stampKey(m->header);
        std::lock_guard<std::mutex> lk(mu_);
        recv_gauge_[key] = now;
        count_gauge_++;
        const auto it = recv_lidar_.find(key);
        if (it != recv_lidar_.end()) {
            lat_total_sum_ += (now - it->second).seconds();
            lat_total_n_++;
        }
        prune(recv_gauge_);
    }

    // End-to-end: от входного кадра до маркера детекции
    // (измеряется на кадрах, где препятствие обнаружено)
    void onMarkers(const MarkerArray::SharedPtr& m) {
        if (m->markers.empty()) return;
        const auto now = this->now();
        const int64_t key = stampKey(m->markers[0].header);
        std::lock_guard<std::mutex> lk(mu_);
        const auto it = recv_lidar_.find(key);
        if (it != recv_lidar_.end()) {
            lat_e2e_sum_ += (now - it->second).seconds();
            lat_e2e_n_++;
        }
    }

    void report() {
        std::lock_guard<std::mutex> lk(mu_);
        const auto now = this->now();
        const double dt = (now - last_report_).seconds();
        if (dt < 0.2) return;

        const double fps_in    = static_cast<double>(count_lidar_ - last_count_lidar_) / dt;
        const double fps_pre   = static_cast<double>(count_pre_   - last_count_pre_)   / dt;
        const double fps_gauge = static_cast<double>(count_gauge_ - last_count_gauge_) / dt;
        const double fps_det   = static_cast<double>(count_det_   - last_count_det_)   / dt;

        const double lat_pre   = lat_pre_n_   ? 1000.0 * lat_pre_sum_   / lat_pre_n_   : 0.0;
        const double lat_total = lat_total_n_ ? 1000.0 * lat_total_sum_ / lat_total_n_ : 0.0;
        const double lat_e2e   = lat_e2e_n_   ? 1000.0 * lat_e2e_sum_   / lat_e2e_n_   : 0.0;

        RCLCPP_INFO(get_logger(),
            "[metrics] fps: in=%.1f pre=%.1f gauge=%.1f det=%.1f | "
            "latency: preproc=%.1fms pipeline=%.1fms e2e=%.1fms | points_in=%u",
            fps_in, fps_pre, fps_gauge, fps_det,
            lat_pre, lat_total, lat_e2e, last_points_in_);

        last_count_lidar_ = count_lidar_;
        last_count_pre_   = count_pre_;
        last_count_gauge_ = count_gauge_;
        last_count_det_   = count_det_;
        lat_pre_sum_ = 0.0;   lat_pre_n_ = 0;
        lat_total_sum_ = 0.0; lat_total_n_ = 0;
        lat_e2e_sum_ = 0.0;   lat_e2e_n_ = 0;
        last_report_ = now;
    }

    static constexpr size_t kMaxEntries = 64;

    std::mutex mu_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_report_;

    std::map<int64_t, rclcpp::Time> recv_lidar_;
    std::map<int64_t, rclcpp::Time> recv_pre_;
    std::map<int64_t, rclcpp::Time> recv_gauge_;

    uint64_t count_lidar_ = 0, count_pre_ = 0, count_gauge_ = 0, count_det_ = 0;
    uint64_t last_count_lidar_ = 0, last_count_pre_ = 0;
    uint64_t last_count_gauge_ = 0, last_count_det_ = 0;

    double lat_pre_sum_ = 0.0;    uint64_t lat_pre_n_ = 0;
    double lat_total_sum_ = 0.0;  uint64_t lat_total_n_ = 0;
    double lat_e2e_sum_ = 0.0;    uint64_t lat_e2e_n_ = 0;

    unsigned last_points_in_ = 0;

    rclcpp::Subscription<PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<PointCloud2>::SharedPtr sub_pre_;
    rclcpp::Subscription<PointCloud2>::SharedPtr sub_gauge_;
    rclcpp::Subscription<MarkerArray>::SharedPtr sub_markers_;
    rclcpp::Subscription<String>::SharedPtr sub_status_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MetricsLogger>());
    rclcpp::shutdown();
    return 0;
}
