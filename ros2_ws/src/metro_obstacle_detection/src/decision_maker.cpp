#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class DecisionMaker : public rclcpp::Node {
public:
    DecisionMaker() : Node("decision_maker") {
        sub_ = create_subscription<std_msgs::msg::String>( \
            "/metro/obstacles_raw", 10, std::bind(&DecisionMaker::obstacleCallback, this, std::placeholders::_1));

        pub_ = create_publisher<std_msgs::msg::String>("/metro/safety_status", 10);

        RCLCPP_INFO(get_logger(), "Decision maker node started");
    }

private:
    void obstacleCallback(const std_msgs::msg::String::SharedPtr msg) {
        std_msgs::msg::String status;

        if (msg->data.find("no_obstacle") != std::string::npos) {
            status.data = "SAFE";
        } else {
            status.data = "DANGER: " + msg->data;
        }

        pub_->publish(status);
        RCLCPP_INFO(get_logger(), "Safety status: %s", status.data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DecisionMaker>());
    rclcpp::shutdown();
    return 0;
}
