#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

// Include the service interface header from psdk_interfaces package
#include "psdk_interfaces/srv/camera_setup_streaming.hpp"

using namespace std::chrono_literals;

class CameraStreamCompressor : public rclcpp::Node
{
public:
  CameraStreamCompressor() : Node("camera_stream_compressor")
  {
    // Create a client for the camera setup streaming service
    camera_setup_client_ = this->create_client<psdk_interfaces::srv::CameraSetupStreaming>(
      "/wrapper/psdk_ros2/camera_setup_streaming");

    // Wait for the service to be available
    while (!camera_setup_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for service /wrapper/psdk_ros2/camera_setup_streaming...");
    }

    // Create the service request and populate its fields
    auto request = std::make_shared<psdk_interfaces::srv::CameraSetupStreaming::Request>();
    request->payload_index = 1;
    request->camera_source = 2; // as per your parameter (e.g. second camera)
    request->start_stop = true; // true to start streaming
    request->decoded_output = true; // true for decoded output

    // Send the service request synchronously
    auto result_future = camera_setup_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
          == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result_future.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Camera stream started successfully.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Camera stream service call returned failure.");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call camera stream service.");
    }

    // Define QoS for sensor data streams
    // This matches the publisher QoS to ensure compatibility:
    // - History: Keep last
    // - Depth: 5
    // - Reliability: Best effort
    // - Durability: Volatile
    auto sensor_qos = rclcpp::SensorDataQoS();

    RCLCPP_INFO(this->get_logger(), "Using SensorDataQoS for camera stream (best_effort reliability)");

    // Subscribe to the main camera stream topic with SensorDataQoS
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/wrapper/psdk_ros2/main_camera_stream", sensor_qos,
      std::bind(&CameraStreamCompressor::imageCallback, this, std::placeholders::_1));

    // Create a publisher with the same SensorDataQoS
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/wrapper/psdk_ros2/compressed_camera_stream", sensor_qos);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // Convert the incoming ROS image message to an OpenCV image (BGR8 encoding)
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

      // Resize or compress the image as needed (here we resize to 640x480)
      cv::Mat resized;
      cv::resize(cv_ptr->image, resized, cv::Size(640, 480));

      // (Optional) Further compression can be done via imencode if you need JPEG/PNG compression.
      // For this example, we simply publish the resized image.

      // Convert back to a ROS image message
      auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", resized).toImageMsg();
      image_pub_->publish(*out_msg);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  // Member variables: service client, image subscriber, and publisher
  rclcpp::Client<psdk_interfaces::srv::CameraSetupStreaming>::SharedPtr camera_setup_client_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraStreamCompressor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}