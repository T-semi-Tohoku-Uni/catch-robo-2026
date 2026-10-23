#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class RedCylinderDetector : public rclcpp::Node
{
public:
  RedCylinderDetector() : Node("red_cylinder_detector")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    diameter_ = declare_parameter<double>("cylinder_diameter_m", 0.066, descriptor);
    const auto length = declare_parameter<double>("cylinder_length_m", 0.170, descriptor);
    if (!std::isfinite(diameter_) || diameter_ < 0.0 ||
      !std::isfinite(length) || length <= 0.0)
    {
      throw std::invalid_argument("Cylinder diameter must be nonnegative and length must be positive");
    }
    // Allow perspective and segmentation variation around the upright silhouette ratio.
    const double nominal_aspect = diameter_ > 0.0 ? length / diameter_ : 0.0;
    hue_width_ = declare_parameter<int>("hue_width", 10, descriptor);
    saturation_min_ = declare_parameter<int>("saturation_min", 100, descriptor);
    value_min_ = declare_parameter<int>("value_min", 60, descriptor);
    min_area_ = declare_parameter<double>("min_area", 300.0, descriptor);
    min_aspect_ = declare_parameter<double>(
      "min_aspect_ratio", diameter_ > 0.0 ? nominal_aspect * 0.65 : 1.2, descriptor);
    max_aspect_ = declare_parameter<double>(
      "max_aspect_ratio", diameter_ > 0.0 ? nominal_aspect * 1.35 : 8.0, descriptor);
    min_fill_ = declare_parameter<double>("min_fill_ratio", 0.55, descriptor);
    kernel_size_ = declare_parameter<int>("morphology_kernel", 5, descriptor);
    publish_debug_ = declare_parameter<bool>("publish_debug_image", true, descriptor);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 75, descriptor);
    if (jpeg_quality_ < 1 || jpeg_quality_ > 100) {
      throw std::invalid_argument("jpeg_quality must be in [1, 100]");
    }
    // Optional pinhole estimate for an upright cylinder of known diameter.
    fx_ = declare_parameter<double>("fx", 0.0, descriptor);
    fy_ = declare_parameter<double>("fy", 0.0, descriptor);
    cx_ = declare_parameter<double>("cx", 0.0, descriptor);
    cy_ = declare_parameter<double>("cy", 0.0, descriptor);
    if (hue_width_ < 0 || hue_width_ > 89 ||
      saturation_min_ < 0 || saturation_min_ > 255 || value_min_ < 0 || value_min_ > 255 ||
      !std::isfinite(min_area_) || min_area_ <= 0.0 ||
      !std::isfinite(min_aspect_) || !std::isfinite(max_aspect_) ||
      min_aspect_ <= 0.0 || max_aspect_ < min_aspect_ ||
      !std::isfinite(min_fill_) || min_fill_ <= 0.0 || min_fill_ > 1.0 ||
      kernel_size_ < 1 || kernel_size_ > 31 || kernel_size_ % 2 == 0)
    {
      throw std::invalid_argument("Invalid HSV, area, aspect, fill or morphology parameters");
    }
    if (!std::isfinite(fx_) || !std::isfinite(fy_) || fx_ < 0.0 || fy_ < 0.0 ||
      !std::isfinite(cx_) || !std::isfinite(cy_) ||
      ((fx_ > 0.0) != (fy_ > 0.0)))
    {
      throw std::invalid_argument("Set both fx and fy positive, or both zero; cx/cy must be finite");
    }
    kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, {kernel_size_, kernel_size_});
    pixel_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("red_cylinder/center_pixel", 10);
    detected_pub_ = create_publisher<std_msgs::msg::Bool>("red_cylinder/detected", 10);
    result_pub_ = create_publisher<std_msgs::msg::String>(
      "red_cylinder/result", rclcpp::QoS(1).best_effort());
    if (diameter_ > 0.0 && fx_ > 0.0 && fy_ > 0.0) {
      position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("red_cylinder/position", 10);
    } else if (diameter_ > 0.0) {
      RCLCPP_WARN(get_logger(),
        "Camera intrinsics unset: publishing pixel coordinates and debug images only. "
        "Set calibrated fx, fy, cx, cy for 3D estimation.");
    }
    if (publish_debug_) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "red_cylinder/debug_image", rclcpp::QoS(1).reliable());
      compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "red_cylinder/debug_image/compressed", rclcpp::QoS(1).best_effort());
    }
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", rclcpp::QoS(1).reliable(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) { process(*message); });
    RCLCPP_INFO(get_logger(), "Listening to %s; center_pixel uses pixels (u right, v down, z=0)",
      subscription_->get_topic_name());
    RCLCPP_INFO(get_logger(),
      "Cylinder diameter=%.1f mm, length=%.1f mm; image height/width range %.2f..%.2f",
      diameter_ * 1000.0, length * 1000.0, min_aspect_, max_aspect_);
  }

private:
  void process(const sensor_msgs::msg::Image & image)
  {
    const auto processing_started = std::chrono::steady_clock::now();
    double jpeg_encode_ms = 0.0;
    try {
      const bool rgb = image.encoding == sensor_msgs::image_encodings::RGB8;
      if (!rgb && image.encoding != sensor_msgs::image_encodings::BGR8) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Expected bgr8 or rgb8 image");
        return;
      }
      if (image.width == 0 || image.height == 0 ||
        image.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
        image.height > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
        image.step < static_cast<uint64_t>(image.width) * 3 ||
        image.data.size() < static_cast<uint64_t>(image.step) * image.height)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid image size/step/data");
        return;
      }
      // OpenCV's Mat API needs a mutable pointer; the view is only read below.
      const cv::Mat view(static_cast<int>(image.height), static_cast<int>(image.width),
        CV_8UC3, const_cast<unsigned char *>(image.data.data()), image.step);
      cv::Mat hsv, low, high, mask;
      cv::cvtColor(view, hsv, rgb ? cv::COLOR_RGB2HSV : cv::COLOR_BGR2HSV);
      cv::inRange(hsv, cv::Scalar(0, saturation_min_, value_min_),
        cv::Scalar(hue_width_, 255, 255), low);
      cv::inRange(hsv, cv::Scalar(179 - hue_width_, saturation_min_, value_min_),
        cv::Scalar(179, 255, 255), high);
      cv::bitwise_or(low, high, mask);
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_);
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_);
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      double best_area = 0.0;
      cv::Rect best_box;
      cv::Point2d center;
      for (const auto & contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < min_area_ || area <= best_area) {
          continue;
        }
        const auto box = cv::boundingRect(contour);
        const double aspect = static_cast<double>(box.height) / box.width;
        const double fill = area / (static_cast<double>(box.width) * box.height);
        if (aspect < min_aspect_ || aspect > max_aspect_ || fill < min_fill_) {
          continue;
        }
        const auto moments = cv::moments(contour);
        if (moments.m00 <= 0.0) {
          continue;
        }
        best_area = area;
        best_box = box;
        center = {moments.m10 / moments.m00, moments.m01 / moments.m00};
      }
      std_msgs::msg::Bool detected;
      detected.data = best_area > 0.0;
      detected_pub_->publish(detected);
      // A stamped result lets the HTTP bridge join coordinates to the exact image.
      std_msgs::msg::String result;
      result.data = "{\"sec\":" + std::to_string(image.header.stamp.sec) +
        ",\"nanosec\":" + std::to_string(image.header.stamp.nanosec) +
        ",\"width\":" + std::to_string(image.width) +
        ",\"height\":" + std::to_string(image.height) +
        ",\"detected\":" + (detected.data ? "true" : "false") +
        ",\"center_pixel\":" + (detected.data ?
        "{\"u\":" + std::to_string(center.x) + ",\"v\":" + std::to_string(center.y) + "}" :
        "null") + "}";
      if (detected.data) {
        geometry_msgs::msg::PointStamped point;
        point.header = image.header;
        point.point.x = center.x;
        point.point.y = center.y;
        point.point.z = 0.0;
        pixel_pub_->publish(point);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "Red cylinder candidate: u=%.1f, v=%.1f px", center.x, center.y);
        // A clipped silhouette has no reliable width for distance estimation.
        if (position_pub_ && best_box.x > 0 && best_box.y > 0 &&
          best_box.x + best_box.width < view.cols &&
          best_box.y + best_box.height < view.rows)
        {
          if (image.header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "3D output needs an optical camera frame_id in the input header");
          } else {
            const double z = fx_ * diameter_ / best_box.width;
            point.point.x = (center.x - cx_) * z / fx_;
            point.point.y = (center.y - cy_) * z / fy_;
            point.point.z = z;
            position_pub_->publish(point);
          }
        }
      }
      if (debug_pub_) {
        cv::Mat debug;
        if (rgb) {
          cv::cvtColor(view, debug, cv::COLOR_RGB2BGR);
        } else {
          debug = view.clone();
        }
        if (detected.data) {
          cv::rectangle(debug, best_box, cv::Scalar(0, 255, 0), 2);
          cv::circle(debug, center, 4, cv::Scalar(0, 255, 255), -1);
          cv::putText(debug, "u=" + std::to_string(static_cast<int>(std::lround(center.x))) +
            " v=" + std::to_string(static_cast<int>(std::lround(center.y))),
            {best_box.x, std::max(15, best_box.y - 5)}, cv::FONT_HERSHEY_SIMPLEX,
            0.5, cv::Scalar(0, 255, 0), 1);
        }
        sensor_msgs::msg::Image output;
        output.header = image.header;
        output.width = image.width;
        output.height = image.height;
        output.encoding = sensor_msgs::image_encodings::BGR8;
        output.step = image.width * 3;
        output.data.assign(debug.data, debug.data + debug.total() * debug.elemSize());
        debug_pub_->publish(output);
        if (compressed_pub_->get_subscription_count() > 0) {
          sensor_msgs::msg::CompressedImage compressed;
          compressed.header = image.header;
          compressed.format = "bgr8; jpeg compressed bgr8";
          const auto encoding_started = std::chrono::steady_clock::now();
          const auto encoded = cv::imencode(".jpg", debug, compressed.data,
            {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
          jpeg_encode_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - encoding_started).count();
          if (encoded)
          {
            compressed_pub_->publish(std::move(compressed));
          }
        }
      }
      const auto processing_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - processing_started).count();
      result.data.pop_back();
      result.data += ",\"jpeg_encode_ms\":" + std::to_string(jpeg_encode_ms) +
        ",\"processing_ms\":" + std::to_string(processing_ms) + "}";
      result_pub_->publish(result);
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "OpenCV: %s", error.what());
    }
  }

  int hue_width_, saturation_min_, value_min_, kernel_size_;
  int jpeg_quality_;
  double min_area_, min_aspect_, max_aspect_, min_fill_;
  double diameter_, fx_, fy_, cx_, cy_;
  bool publish_debug_;
  cv::Mat kernel_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pixel_pub_, position_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = EXIT_SUCCESS;
  try {
    rclcpp::spin(std::make_shared<RedCylinderDetector>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("red_cylinder_detector"), "%s", error.what());
    status = EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return status;
}
