#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/utilities.hpp>

class FollowLine : public rclcpp::Node {

private:
    using Twist = geometry_msgs::msg::Twist;
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;


public:
    explicit FollowLine() : Node("follow_line_node") {
        cmd_vel_pub_ = this->create_publisher<Twist>("cmd_vel", 10);
        auto msg = Twist();
        msg.linear.x = 1.0;
        cmd_vel_pub_->publish(msg);
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FollowLine>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();

    return 0;
}