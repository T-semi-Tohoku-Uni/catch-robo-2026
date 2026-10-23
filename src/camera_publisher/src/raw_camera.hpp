#pragma once

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <opencv2/imgproc.hpp>

// Jetson IMX219 only: RG10 arrives in the upper ten bits of a 16-bit word.
// Owns the V4L2 stream independently of OpenCV's VideoCapture conversions.
class RawCamera
{
public:
  RawCamera() = default;
  RawCamera(const RawCamera &) = delete;
  RawCamera & operator=(const RawCamera &) = delete;
  ~RawCamera()
  {
    if (streaming_) {
      auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      ioctl(fd_, VIDIOC_STREAMOFF, &type);
    }
    for (const auto & buffer : buffers_) {
      munmap(buffer.address, buffer.length);
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  void open(
    const std::string & device, int mode, double fps, int exposure, int gain,
    int output_width, int output_height)
  {
    static constexpr int widths[] = {3280, 3280, 1920, 1640, 1280};
    static constexpr int heights[] = {2464, 1848, 1080, 1232, 720};
    static constexpr double max_fps[] = {21, 28, 30, 30, 60};
    if (mode < 0 || mode > 4 || fps > max_fps[mode]) {
      throw std::invalid_argument("Invalid IMX219 raw_sensor_mode or fps for that mode");
    }
    // On this sensor, asking mode 4 for 15 FPS stalls capture. Keep the sensor
    // at a working rate and let the ROS timer select the output frame rate.
    capture_fps_ = mode == 4 && fps <= 30.0 ? 30.0 : max_fps[mode];
    if (exposure < 13 || exposure > std::floor(1000000.0 / capture_fps_)) {
      throw std::invalid_argument("exposure_us must fit within the raw sensor frame period");
    }
    cv::setNumThreads(2);
    output_width_ = output_width;
    output_height_ = output_height;
    fd_ = ::open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      fail("open " + device);
    }
    v4l2_capability capability{};
    checked(VIDIOC_QUERYCAP, &capability, "QUERYCAP");
    if (std::string(reinterpret_cast<char *>(capability.driver)) != "tegra-video" ||
      std::string(reinterpret_cast<char *>(capability.card)).find("imx219") == std::string::npos)
    {
      throw std::runtime_error("raw backend supports Jetson IMX219 only");
    }
    set_control("sensor_mode", mode);
    set_control("bypass_mode", 0);
    set_control("override_enable", 1);

    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = widths[mode];
    format.fmt.pix.height = heights[mode];
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB10;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    checked(VIDIOC_S_FMT, &format, "S_FMT RG10");
    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_SRGGB10 ||
      format.fmt.pix.bytesperline < format.fmt.pix.width * 2)
    {
      throw std::runtime_error("Camera did not accept unpacked RG10 format");
    }
    width_ = format.fmt.pix.width;
    height_ = format.fmt.pix.height;
    stride_ = format.fmt.pix.bytesperline;
    set_control("frame_rate", std::llround(capture_fps_ * 1000000));
    set_control("gain", gain);
    set_control("exposure", exposure);

    v4l2_requestbuffers request{};
    request.count = 4;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    checked(VIDIOC_REQBUFS, &request, "REQBUFS");
    if (request.count == 0) {
      throw std::runtime_error("No V4L2 buffers available");
    }
    buffers_.reserve(request.count);
    for (unsigned int index = 0; index < request.count; ++index) {
      v4l2_buffer buffer{};
      buffer.type = request.type;
      buffer.memory = request.memory;
      buffer.index = index;
      checked(VIDIOC_QUERYBUF, &buffer, "QUERYBUF");
      if (buffer.length < stride_ * static_cast<size_t>(height_)) {
        throw std::runtime_error("V4L2 buffer is smaller than the image");
      }
      void * address = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE,
        MAP_SHARED, fd_, buffer.m.offset);
      if (address == MAP_FAILED) {
        fail("mmap");
      }
      buffers_.push_back({address, buffer.length});
      checked(VIDIOC_QBUF, &buffer, "QBUF");
    }
    auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    checked(VIDIOC_STREAMON, &type, "STREAMON");
    streaming_ = true;
  }

  bool read(cv::Mat & image)
  {
    pollfd descriptor{fd_, POLLIN, 0};
    if (poll(&descriptor, 1, 500) <= 0 || !(descriptor.revents & POLLIN)) {
      return false;
    }
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (call(VIDIOC_DQBUF, &buffer) < 0) {
      if (errno == EAGAIN) {
        return false;
      }
      fail("DQBUF");
    }
    if (buffer.index >= buffers_.size()) {
      throw std::runtime_error("Invalid V4L2 buffer index");
    }
    // Drain older queued frames when publishing more slowly than the sensor.
    for (size_t index = 1; index < buffers_.size(); ++index) {
      v4l2_buffer next{};
      next.type = buffer.type;
      next.memory = buffer.memory;
      if (call(VIDIOC_DQBUF, &next) < 0) {
        if (errno == EAGAIN) {
          break;
        }
        fail("DQBUF latest frame");
      }
      checked(VIDIOC_QBUF, &buffer, "QBUF older frame");
      buffer = next;
      if (buffer.index >= buffers_.size()) {
        throw std::runtime_error("Invalid V4L2 buffer index");
      }
    }
    if ((buffer.flags & V4L2_BUF_FLAG_ERROR) ||
      buffer.bytesused < stride_ * static_cast<size_t>(height_))
    {
      checked(VIDIOC_QBUF, &buffer, "QBUF");
      return false;
    }
    cv::Mat raw;
    try {
      raw = cv::Mat(height_, width_, CV_16UC1,
        buffers_[buffer.index].address, stride_).clone();
    } catch (...) {
      call(VIDIOC_QBUF, &buffer);
      throw;
    }
    checked(VIDIOC_QBUF, &buffer, "QBUF");

    // Remove the sensor black level (64/1023), then demosaic in linear light.
    cv::Mat linear;
    raw.convertTo(linear, CV_16U, 1.0, -4096.0);
    cv::cvtColor(linear, linear, cv::COLOR_BayerRGGB2BGR);
    cv::resize(linear, linear, cv::Size(output_width_, output_height_), 0, 0, cv::INTER_AREA);
    std::vector<cv::Mat> channels;
    cv::split(linear, channels);
    const auto means = cv::mean(linear);
    const double target = std::max({means[0], means[1], means[2], 1.0});
    for (int channel = 0; channel < 3; ++channel) {
      // Simple grey-world white balance; does not replace calibrated ISP tuning.
      const double gain = std::clamp(target / std::max(means[channel], 1.0), 1.0, 4.0);
      channels[channel].convertTo(channels[channel], CV_8U, gain * 255.0 / 61376.0);
    }
    cv::merge(channels, image);
    cv::Mat gamma(1, 256, CV_8U);
    for (int value = 0; value < 256; ++value) {
      gamma.at<unsigned char>(value) = cv::saturate_cast<unsigned char>(
        std::pow(value / 255.0, 1.0 / 2.2) * 255.0);
    }
    cv::LUT(image, gamma, image);
    return true;
  }

  double capture_fps() const {return capture_fps_;}

private:
  [[noreturn]] void fail(const std::string & operation) const
  {
    throw std::runtime_error("Raw camera " + operation + ": " + std::strerror(errno));
  }

  int call(unsigned long request, void * argument) const
  {
    int result;
    do {
      result = ioctl(fd_, request, argument);
    } while (result < 0 && errno == EINTR);
    return result;
  }

  void checked(unsigned long request, void * argument, const std::string & operation) const
  {
    if (call(request, argument) < 0) {
      fail(operation);
    }
  }

  void set_control(const std::string & name, int64_t value)
  {
    v4l2_query_ext_ctrl query{};
    query.id = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
    while (call(VIDIOC_QUERY_EXT_CTRL, &query) == 0) {
      // v4l2-ctl presents "Sensor Mode" as sensor_mode, etc.
      std::string control_name(query.name);
      std::transform(control_name.begin(), control_name.end(), control_name.begin(),
        [](unsigned char ch) {return ch == ' ' ? '_' : static_cast<char>(std::tolower(ch));});
      if (name == control_name) {
        v4l2_ext_control control{};
        control.id = query.id;
        if (query.type == V4L2_CTRL_TYPE_INTEGER64) {
          control.value64 = value;
        } else {
          control.value = static_cast<int>(value);
        }
        v4l2_ext_controls controls{};
        controls.ctrl_class = V4L2_CTRL_ID2CLASS(query.id);
        controls.count = 1;
        controls.controls = &control;
        checked(VIDIOC_S_EXT_CTRLS, &controls, "set " + name);
        return;
      }
      query.id |= V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
    }
    throw std::runtime_error("Raw camera control not found: " + name);
  }

  struct Buffer {void * address; size_t length;};
  int fd_{-1};
  bool streaming_{false};
  int width_{0}, height_{0}, output_width_{0}, output_height_{0};
  size_t stride_{0};
  double capture_fps_{0};
  std::vector<Buffer> buffers_;
};
