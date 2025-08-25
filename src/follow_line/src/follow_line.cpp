#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/twist.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <opencv2/core/types.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>

class FollowLine : public rclcpp::Node {

private:
  static constexpr const char *NODE_NAME = "follow_line_node";
  static constexpr const char *VEL_TOPIC_NAME = "cmd_vel";
  static constexpr const char *IMAGE_PUB_NAME = "gazebo_camera_circle";
  static constexpr const char *IMAGE_SUB_NAME = "gazebo_camera";

  using Twist = geometry_msgs::msg::Twist;
  using Image = sensor_msgs::msg::Image;

  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Publisher<Image>::SharedPtr image_pub_;

  /**
   * @brief Callback that draws a circle in the image received and publishs it to IMAGE_PUB_NAME
   * 
   * @param image 
   */
  void image_callback(const Image::SharedPtr &image) {
    cv_bridge::CvImagePtr cv_ptr;

    //* Convert the sensor_msgs to cv::Mat
    try {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat &cv_image = cv_ptr->image;

    //* Write a centered circle with diameter img_width/4 of color red.
    cv::circle(cv_image, cv::Point(image->width / 2, image->height / 2),
               image->width / 4, cv::Scalar(0, 0, 255), 3);

    image_pub_->publish(*cv_ptr->toImageMsg());

    auto msg = Twist();
    msg.linear.x = 1.0;
    cmd_vel_pub_->publish(msg);
  }

public:
  explicit FollowLine() : Node(NODE_NAME) {
    //* Initialize subs and pubs
    cmd_vel_pub_ = this->create_publisher<Twist>(VEL_TOPIC_NAME, 10);

    image_pub_ = this->create_publisher<Image>(IMAGE_PUB_NAME, 10);
    image_sub_ = this->create_subscription<Image>(
        IMAGE_SUB_NAME, 10,
        [this](Image::SharedPtr image) { this->image_callback(image); });
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<FollowLine>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}