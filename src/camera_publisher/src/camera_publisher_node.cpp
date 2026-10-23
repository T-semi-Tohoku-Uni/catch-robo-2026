#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/videoio.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include "raw_camera.hpp"

class CameraPublisher : public rclcpp::Node
{
public:
  CameraPublisher() : Node("camera_publisher")
  {
    // Camera settings are applied at startup only.
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto device = declare_parameter<std::string>("device", "/dev/video0", descriptor);
    auto backend = declare_parameter<std::string>("backend", "auto", descriptor);
    const auto video_path = declare_parameter<std::string>("video_path", "", descriptor);
    video_loop_ = declare_parameter<bool>("video_loop", true, descriptor);
    const auto sensor_id = declare_parameter<int>("sensor_id", 0, descriptor);
    const auto width = declare_parameter<int>("width", 640, descriptor);
    const auto height = declare_parameter<int>("height", 480, descriptor);
    const auto fps = declare_parameter<double>("fps", 30.0, descriptor);
    const auto raw_mode = declare_parameter<int>("raw_sensor_mode", 4, descriptor);
    const auto exposure = declare_parameter<int>("exposure_us", 20000, descriptor);
    const auto gain = declare_parameter<int>("gain", 64, descriptor);
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_optical_frame", descriptor);

    if (width <= 0 || height <= 0 || !std::isfinite(fps) || fps <= 0.0 || fps > 1000.0) {
      throw std::invalid_argument("width/height must be positive; fps must be in (0, 1000]");
    }
    if (backend == "auto") {
      // This Jetson's IMX219 works through raw V4L2 even when Argus times out.
      const auto name = device.substr(device.find_last_of('/') + 1);
      std::ifstream sysfs("/sys/class/video4linux/" + name + "/name");
      std::string driver;
      std::getline(sysfs, driver);
      backend = driver.find("vi-output") != std::string::npos ? "argus" : "v4l2";
      if (driver.find("vi-output") != std::string::npos &&
        driver.find("imx219") != std::string::npos)
      {
        backend = "raw";
      }
    }
    video_ = backend == "video";
    if (video_) {
      if (video_path.empty() || !capture_.open(video_path)) {
        throw std::runtime_error("Could not open video_path: " + video_path);
      }
    } else if (backend == "raw") {
      raw_camera_ = std::make_unique<RawCamera>();
      raw_camera_->open(device, raw_mode, fps, exposure, gain, width, height);
    } else if (backend == "argus") {
      if (sensor_id < 0) {
        throw std::invalid_argument("sensor_id must be nonnegative");
      }
      const auto fps_numerator = static_cast<int>(std::lround(fps * 1000.0));
      if (fps_numerator == 0) {
        throw std::invalid_argument("Argus fps must be at least 0.001");
      }
      const std::string pipeline =
        "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) +
        " ! video/x-raw(memory:NVMM),format=NV12,framerate=" +
        std::to_string(fps_numerator) + "/1000"
        " ! nvvidconv ! video/x-raw,format=BGRx,width=" + std::to_string(width) +
        ",height=" + std::to_string(height) +
        " ! videoconvert ! video/x-raw,format=BGR"
        " ! appsink drop=true max-buffers=1 sync=false";
      RCLCPP_INFO(get_logger(), "Opening Argus sensor %s", std::to_string(sensor_id).c_str());
      if (!capture_.open(pipeline, cv::CAP_GSTREAMER)) {
        throw std::runtime_error(
          "Could not open Argus camera. Check sensor_id, nvargus-daemon and GStreamer plugins.");
      }
      cv::Mat initial_frame;
      if (!capture_.read(initial_frame) || initial_frame.empty()) {
        throw std::runtime_error(
          "Argus produced no image. For SSH sessions, launch with env -u DISPLAY. "
          "If the sensor timed out, restart nvargus-daemon before retrying.");
      }
    } else if (backend == "v4l2") {
      if (!capture_.open(device, cv::CAP_V4L2)) {
        throw std::runtime_error("Could not open camera: " + device);
      }
      capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
      capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
      capture_.set(cv::CAP_PROP_FPS, fps);
    } else {
      throw std::invalid_argument("backend must be auto, raw, argus, v4l2 or video");
    }

    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "camera/image_raw", rclcpp::QoS(1).reliable());
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / fps));
    timer_ = create_wall_timer(period, [this]() { publish_frame(); });

    RCLCPP_INFO(
      get_logger(), "Camera backend=%s: %.0fx%.0f, camera FPS %.2f, publish target %.2f Hz -> %s",
      backend.c_str(), raw_camera_ ? static_cast<double>(width) : capture_.get(cv::CAP_PROP_FRAME_WIDTH),
      raw_camera_ ? static_cast<double>(height) : capture_.get(cv::CAP_PROP_FRAME_HEIGHT),
      raw_camera_ ? raw_camera_->capture_fps() : capture_.get(cv::CAP_PROP_FPS), fps,
      publisher_->get_topic_name());
  }

private:
  void publish_frame()
  {
    try {
      cv::Mat frame;
      bool received = raw_camera_ ? raw_camera_->read(frame) : capture_.read(frame);
      if (!received && video_ && video_loop_) {
        capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
        received = capture_.read(frame);
      }
      if (!received || frame.empty()) {
        if (!rclcpp::ok()) {
          return;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Could not read camera frame");
        return;
      }
      if (frame.type() != CV_8UC3) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "Expected an 8-bit BGR camera frame");
        return;
      }
      auto message = std::make_unique<sensor_msgs::msg::Image>();
      message->header.stamp = now();
      message->header.frame_id = frame_id_;
      message->height = frame.rows;
      message->width = frame.cols;
      message->encoding = sensor_msgs::image_encodings::BGR8;
      message->is_bigendian = false;  // Each channel is one byte.
      message->step = frame.cols * frame.elemSize();
      message->data.resize(message->step * message->height);
      // Copy each row so non-contiguous OpenCV frames are supported too.
      for (int row = 0; row < frame.rows; ++row) {
        std::copy_n(
          frame.ptr<unsigned char>(row), message->step,
          message->data.data() + static_cast<size_t>(row) * message->step);
      }
      publisher_->publish(std::move(message));
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "OpenCV: %s", error.what());
    }
  }

  cv::VideoCapture capture_;
  bool video_{false}, video_loop_{true};
  std::unique_ptr<RawCamera> raw_camera_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = EXIT_SUCCESS;
  try {
    rclcpp::spin(std::make_shared<CameraPublisher>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("camera_publisher"), "%s", error.what());
    status = EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return status;
}
