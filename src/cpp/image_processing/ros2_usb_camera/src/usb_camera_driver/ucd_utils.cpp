/**
 * ROS 2 USB Camera Driver node auxiliary routines.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 4, 2022
 */

/**
 * Copyright © 2022 Intelligent Systems Lab
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <cfloat>
#include <cstdint>
#include <cstring>

#include <usb_camera_driver/usb_camera_driver.hpp>

namespace USBCameraDriver
{

/**
 * @brief Converts a frame into an Image message.
 *
 * @param frame cv::Mat storing the frame.
 * @return Shared pointer to a new Image message.
 */
Image::SharedPtr CameraDriverNode::frame_to_msg(cv::Mat & frame)
{
  // Resize the frame as per node parameters
  if (frame.rows != image_width_ || frame.cols != image_height_) {
    cv::resize(frame, frame, cv::Size(image_width_, image_height_));
  }

  // Allocate new image message
  auto ros_image = std::make_shared<Image>();

  // Set frame-relevant image contents
  ros_image->set__width(frame.cols);
  ros_image->set__height(frame.rows);
  ros_image->set__encoding(sensor_msgs::image_encodings::BGR8);
  ros_image->set__step(frame.cols * frame.elemSize());

  // Check data endianness
  int num = 1;
  ros_image->set__is_bigendian(false);

  // Copy frame data (this avoids the obsolete cv_bridge)
  size_t size = ros_image->step * frame.rows;
  ros_image->data.resize(size);
  std::memcpy(ros_image->data.data(), frame.data, size);

  return ros_image;
}

/**
 * @brief Initializes node parameters.
 */
void CameraDriverNode::init_parameters()
{
  // Register parameter updates callback
  on_set_params_chandle_ = this->add_on_set_parameters_callback(
    std::bind(
      &CameraDriverNode::on_set_parameters_callback,
      this,
      std::placeholders::_1));

  // Base topic name
  declare_string_parameter(
    "base_topic_name",
    "camera",
    "image_transport base topic name.",
    "Cannot be changed.",
    true,
    base_topic_name_descriptor_);

  // Best-effort QoS flag
  declare_bool_parameter(
    "best_effort_qos",
    false,
    "Best-effort QoS flag.",
    "Cannot be changed.",
    true,
    base_topic_name_descriptor_);

  // Brightness
  declare_double_parameter(
    "brightness",
    0.0, -DBL_MAX, DBL_MAX, 0.0,
    "Image brightness.",
    "Camera-dependent, 0.0 means auto.",
    false,
    exposure_descriptor_);

  // Camera calibration file URL
  declare_string_parameter(
    "camera_calibration_file",
    "file://config/camera.yaml",
    "Camera calibration file URL.",
    "Cannot be changed.",
    true,
    camera_calibration_file_descriptor_);

  // Camera device ID
  declare_int_parameter(
    "camera_id",
    0, 0, INT64_MAX, 1,
    "Camera device ID.",
    "Cannot be changed.",
    true,
    camera_id_descriptor_);

  // Camera name
  declare_string_parameter(
    "camera_name",
    "camera",
    "Camera name from the configuration file.",
    "Cannot be changed.",
    true,
    camera_name_descriptor_);

  // Exposure
  declare_double_parameter(
    "exposure",
    0.0, -DBL_MAX, DBL_MAX, 0.0,
    "Camera exposure.",
    "Camera-dependent, 0.0 means auto.",
    false,
    exposure_descriptor_);

  // RViz frame ID
  declare_string_parameter(
    "frame_id",
    "map",
    "RViz frame ID.",
    "Cannot be changed.",
    true,
    frame_id_descriptor_);

  // Camera FPS
  declare_int_parameter(
    "fps",
    10, 1, 60, 1,
    "Camera FPS.",
    "Cannot be changed.",
    true,
    fps_descriptor_);

  // Image height
  declare_int_parameter(
    "image_height",
    480, 1, INT64_MAX, 1,
    "Image height.",
    "Cannot be changed.",
    true,
    image_height_descriptor_);

  // Image width
  declare_int_parameter(
    "image_width",
    640, 1, INT64_MAX, 1,
    "Image width.",
    "Cannot be changed.",
    true,
    image_width_descriptor_);

  // Camera flipped flag
  declare_bool_parameter(
    "is_flipped",
    false,
    "Camera flipped flag.",
    "Can be changed at run time.",
    false,
    is_flipped_descriptor_);

  // WB temperature
  declare_double_parameter(
    "wb_temperature",
    0.0, -DBL_MAX, DBL_MAX, 0.0,
    "White balance color temperature.",
    "Can be changed at runtime, 0.0 means auto.",
    false,
    wb_temperature_descriptor_);
}

/**
 * @brief Routine to declare a boolean node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param descriptor Parameter descriptor.
 */
void CameraDriverNode::declare_bool_parameter(
  std::string && name,
  bool default_val,
  std::string && desc, std::string && constraints,
  bool read_only, ParameterDescriptor & descriptor)
{
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_BOOL);
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  this->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare a 64-bit floating point node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param from Floating point range initial value.
 * @param to Floating point range final value.
 * @param step Floating point range step.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param descriptor Parameter descriptor.
 */
void CameraDriverNode::declare_double_parameter(
  std::string && name,
  double default_val, double from, double to, double step,
  std::string && desc, std::string && constraints,
  bool read_only, ParameterDescriptor & descriptor)
{
  FloatingPointRange param_range{};
  param_range.set__from_value(from);
  param_range.set__to_value(to);
  param_range.set__step(step);
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_DOUBLE);
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  descriptor.set__floating_point_range({param_range});
  this->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare an integer node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param from Integer range initial value.
 * @param to Integer range final value.
 * @param step Integer range step.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param descriptor Parameter descriptor.
 */
void CameraDriverNode::declare_int_parameter(
  std::string && name,
  int64_t default_val, int64_t from, int64_t to, int64_t step,
  std::string && desc, std::string && constraints,
  bool read_only, ParameterDescriptor & descriptor)
{
  IntegerRange param_range{};
  param_range.set__from_value(from);
  param_range.set__to_value(to);
  param_range.set__step(step);
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_INTEGER);
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  descriptor.set__integer_range({param_range});
  this->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Routine to declare a string node parameter.
 *
 * @param name Parameter name.
 * @param default_val Default value.
 * @param desc Parameter description.
 * @param constraints Additional value constraints.
 * @param read_only Read-only internal flag.
 * @param descriptor Parameter descriptor.
 */
void CameraDriverNode::declare_string_parameter(
  std::string && name,
  std::string && default_val,
  std::string && desc, std::string && constraints,
  bool read_only, ParameterDescriptor & descriptor)
{
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_STRING);
  descriptor.set__description(desc);
  descriptor.set__additional_constraints(constraints);
  descriptor.set__read_only(read_only);
  descriptor.set__dynamic_typing(false);
  this->declare_parameter(name, default_val, descriptor);
}

/**
 * @brief Parameters update validation callback.
 *
 * @param params Vector of parameters for which a change has been requested.
 * @return Operation result in SetParametersResult message.
 */
SetParametersResult CameraDriverNode::on_set_parameters_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  // Initialize result object
  SetParametersResult res{};
  res.set__successful(true);
  res.set__reason("");

  // First, check if each update is feasible
  // Initial checks must be added here!
  for (const rclcpp::Parameter & p : params) {
    // Base topic name
    if (p.get_name() == "base_topic_name") {
      if (p.get_type() != ParameterType::PARAMETER_STRING) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for base_topic_name");
        break;
      }
      continue;
    }

    // Best-effort QoS flag
    if (p.get_name() == "best_effort_qos") {
      if (p.get_type() != ParameterType::PARAMETER_BOOL) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for best_effort_qos");
        break;
      }
      continue;
    }

    // Brightness
    if (p.get_name() == "brightness") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for brightness");
        break;
      }
      continue;
    }

    // Camera calibration file URL
    if (p.get_name() == "camera_calibration_file") {
      if (p.get_type() != ParameterType::PARAMETER_STRING) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for camera_calibration_file");
        break;
      }
      continue;
    }

    // Camera device ID
    if (p.get_name() == "camera_id") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for camera_id");
        break;
      }
      continue;
    }

    // Camera name
    if (p.get_name() == "camera_name") {
      if (p.get_type() != ParameterType::PARAMETER_STRING) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for camera_name");
        break;
      }
      continue;
    }

    // Exposure
    if (p.get_name() == "exposure") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for exposure");
        break;
      }
      continue;
    }

    // RViz frame ID
    if (p.get_name() == "frame_id") {
      if (p.get_type() != ParameterType::PARAMETER_STRING) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for frame_id");
        break;
      }
      continue;
    }

    // Camera FPS
    if (p.get_name() == "fps") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for fps");
        break;
      }
      continue;
    }

    // Image height
    if (p.get_name() == "image_height") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for image_height");
        break;
      }
      continue;
    }

    // Image width
    if (p.get_name() == "image_width") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for image_width");
        break;
      }
      continue;
    }

    // Camera flipped flag
    if (p.get_name() == "is_flipped") {
      if (p.get_type() != ParameterType::PARAMETER_BOOL) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for is_flipped");
        break;
      }
      continue;
    }

    // WB temperature
    if (p.get_name() == "wb_temperature") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for wb_temperature");
        break;
      }
      continue;
    }
  }
  if (!res.successful) {
    return res;
  }

  // Then, do what is necessary to update each parameter
  // Add ad-hoc update procedures must be added here!
  for (const rclcpp::Parameter & p : params) {
    // Base topic name
    if (p.get_name() == "base_topic_name") {
      RCLCPP_INFO(
        this->get_logger(),
        "base_topic_name: %s",
        p.as_string().c_str());
      continue;
    }

    // Best-effort QoS flag
    if (p.get_name() == "best_effort_qos") {
      RCLCPP_INFO(
        this->get_logger(),
        "best_effort_qos: %s",
        p.as_bool() ? "true" : "false");
      continue;
    }

    // Brightness
    if (p.get_name() == "brightness") {
      if (video_cap_.isOpened() && !video_cap_.set(cv::CAP_PROP_BRIGHTNESS, p.as_double())) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set camera brightness");
        res.set__successful(false);
        res.set__reason("cv::VideoCapture::set(CAP_PROP_BRIGHTNESS) failed");
        break;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "brightness: %f",
        p.as_double());
      continue;
    }

    // Camera calibration file URL
    if (p.get_name() == "camera_calibration_file") {
      RCLCPP_INFO(
        this->get_logger(),
        "camera_calibration_file: %s",
        p.as_string().c_str());
      continue;
    }

    // Camera device ID
    if (p.get_name() == "camera_id") {
      RCLCPP_INFO(
        this->get_logger(),
        "camera_id: %ld",
        p.as_int());
      continue;
    }

    // Camera name
    if (p.get_name() == "camera_name") {
      RCLCPP_INFO(
        this->get_logger(),
        "camera_name: %s",
        p.as_string().c_str());
      continue;
    }

    // Exposure
    if (p.get_name() == "exposure") {
      if (video_cap_.isOpened()) {
        bool success;
        if (p.as_double() == 0.0) {
          success = video_cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 3.0);
          if (!success) {
            res.set__successful(false);
            res.set__reason("cv::VideoCapture::set(CAP_PROP_AUTO_EXPOSURE, 3.0) failed");
            RCLCPP_ERROR(this->get_logger(), "Failed to set camera exposure");
            break;
          }
        } else {
          success = video_cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);
          if (!success) {
            res.set__successful(false);
            res.set__reason("cv::VideoCapture::set(CAP_PROP_AUTO_EXPOSURE, 0.75) failed");
            RCLCPP_ERROR(this->get_logger(), "Failed to set camera exposure");
            break;
          }
          success = video_cap_.set(cv::CAP_PROP_EXPOSURE, p.as_double());
          if (!success) {
            res.set__successful(false);
            res.set__reason("cv::VideoCapture::set(CAP_PROP_EXPOSURE) failed");
            RCLCPP_ERROR(this->get_logger(), "Failed to set camera exposure");
            break;
          }
        }
      }
      RCLCPP_INFO(
        this->get_logger(),
        "exposure: %f",
        p.as_double());
      continue;
    }

    // RViz frame ID
    if (p.get_name() == "frame_id") {
      frame_id_ = p.as_string();
      RCLCPP_INFO(
        this->get_logger(),
        "frame_id: %s",
        frame_id_.c_str());
      continue;
    }

    // Camera FPS
    if (p.get_name() == "fps") {
      fps_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "fps: %ld",
        fps_);
      continue;
    }

    // Image height
    if (p.get_name() == "image_height") {
      image_height_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "image_height: %ld",
        image_height_);
      continue;
    }

    // Image width
    if (p.get_name() == "image_width") {
      image_width_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "image_width: %ld",
        image_width_);
      continue;
    }

    // Camera flipped flag
    if (p.get_name() == "is_flipped") {
      is_flipped_ = p.as_bool();
      RCLCPP_INFO(
        this->get_logger(),
        "is_flipped: %s",
        is_flipped_ ? "true" : "false");
      continue;
    }

    // WB temperature
    if (p.get_name() == "wb_temperature") {
      if (video_cap_.isOpened()) {
        bool success;
        if (p.as_double() == 0.0) {
          success = video_cap_.set(cv::CAP_PROP_AUTO_WB, 1.0);
          if (!success) {
            res.set__successful(false);
            res.set__reason("cv::VideoCapture::set(CAP_PROP_AUTO_WB, 1.0) failed");
            RCLCPP_ERROR(this->get_logger(), "Failed to enable auto WB");
            break;
          }
        } else {
          success = video_cap_.set(cv::CAP_PROP_AUTO_WB, 0.0);
          if (!success) {
            res.set__successful(false);
            res.set__reason("cv::VideoCapture::set(CAP_PROP_AUTO_WB, 0.0) failed");
            RCLCPP_ERROR(this->get_logger(), "Failed to disable auto WB");
            break;
          }
          success = video_cap_.set(cv::CAP_PROP_WB_TEMPERATURE, p.as_double());
          if (!success) {
            res.set__successful(false);
            res.set__reason("cv::VideoCapture::set(CAP_PROP_WB_TEMPERATURE) failed");
            RCLCPP_ERROR(this->get_logger(), "Failed to set WB temperature");
            break;
          }
        }
      }
      RCLCPP_INFO(
        this->get_logger(),
        "wb_temperature: %f",
        p.as_double());
      continue;
    }
  }

  return res;
}

} // namespace USBCameraDriver
