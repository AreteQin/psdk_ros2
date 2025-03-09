#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"

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
    request->camera_source = 2;
    request->start_stop = true;
    request->decoded_output = true;

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

    // Configure image transport (instead of direct publishers/subscribers)
    image_transport_ = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());

    // Subscribe to the main camera stream
    image_sub_ = image_transport_->subscribe(
      "/wrapper/psdk_ros2/main_camera_stream", 5,
      std::bind(&CameraStreamCompressor::imageCallback, this, std::placeholders::_1));

    // Create a publisher
    image_pub_ = image_transport_->advertise("/wrapper/psdk_ros2/compressed_camera_stream", 5);

    // Set compression parameters (optional)
    this->declare_parameter("jpeg_quality", 80);
    this->declare_parameter("png_level", 6);

    RCLCPP_INFO(this->get_logger(), "Video compression node initialized with image_transport");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    try {
      // We can optionally resize the image before passing to image_transport
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");

      cv::Mat resized;
      cv::resize(cv_ptr->image, resized, cv::Size(640, 480));

      // Convert back to ROS message and publish
      // image_transport will handle compression based on subscription type
      auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", resized).toImageMsg();
      image_pub_.publish(out_msg);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  // Member variables
  rclcpp::Client<psdk_interfaces::srv::CameraSetupStreaming>::SharedPtr camera_setup_client_;
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraStreamCompressor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}