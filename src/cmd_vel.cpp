#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class PublishTwist : public rclcpp::Node {
public:
    PublishTwist() : Node("publish_twist") {
        position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller", 10);
        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&PublishTwist::cmd_vel_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto position_cmd = std::make_shared<std_msgs::msg::Float64MultiArray>();
        auto velocity_cmd = std::make_shared<std_msgs::msg::Float64MultiArray>();

        double vel = msg->linear.x;
        double angle = msg->angular.z;

        position_cmd->data = {angle, angle};
        velocity_cmd->data = {vel, vel};

        position_publisher_->publish(*position_cmd);
        velocity_publisher_->publish(*velocity_cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublishTwist>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
