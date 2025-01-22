#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// Test
class CameraPublisherNode : public rclcpp::Node {
  public:
    CameraPublisherNode()
        : Node("camera_publisher") {
      std::cout << "test";
      publisher_ =
          this->create_publisher<sensor_msgs::msg::Image>("camera_image", 10);

      if (!cap_.open(0)) {
        RCLCPP_ERROR(this->get_logger(), "Could not open the camera");
        rclcpp::shutdown();
      }

      run();
    }

  private:
    void run() {
      while (true) {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
          RCLCPP_WARN(this->get_logger(), "Captured empty frame");
          return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                       .toImageMsg();
        publisher_->publish(*msg);
      }
    }
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::VideoCapture cap_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
