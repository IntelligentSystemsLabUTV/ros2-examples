/**
 * Aruco Detector node auxiliary functions.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * August 16, 2022
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

#include <aruco_detector/aruco_detector.hpp>

namespace ArucoDetector
{

/**
 * @brief Converts a frame into an Image message.
 *
 * @param frame cv::Mat storing the frame.
 * @return Shared pointer to a new Image message.
 */
Image::SharedPtr ArucoDetectorNode::frame_to_msg(cv::Mat & frame)
{
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
  memcpy(ros_image->data.data(), frame.data, size);

  return ros_image;
}

/**
 * @brief Function to round spatial measure to a fixed precision.
 *
 * @param num Value to round.
 * @param prec 100 -> centimeter, 1000 -> mm
 * @return Rounded value.
 */
float ArucoDetectorNode::round_space(float num, float prec)
{
  num *= prec;
  num = floor(num);
  num /= prec;

  return num;
}

/**
 * @brief Function to round angular measure to a fixed precision.
 *
 * @param num Value to round.
 * @param prec 10 -> tenth of a degree
 * @return Rounded value.
 */
float ArucoDetectorNode::round_angle(float num, float prec)
{
  num = num * 180.0 / M_PIf32;

  num *= prec;
  num = floor(num);
  num /= prec;

  return num * M_PIf32 / 180.0;
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
void ArucoDetectorNode::declare_bool_parameter(
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
void ArucoDetectorNode::declare_double_parameter(
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
void ArucoDetectorNode::declare_int_parameter(
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
 * @brief Routine to declare an integer array node parameter.
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
void ArucoDetectorNode::declare_int_array_parameter(
  std::string && name,
  std::vector<int64_t> && default_val, int64_t from, int64_t to, int64_t step,
  std::string && desc, std::string && constraints,
  bool read_only, ParameterDescriptor & descriptor)
{
  IntegerRange param_range{};
  param_range.set__from_value(from);
  param_range.set__to_value(to);
  param_range.set__step(step);
  descriptor.set__name(name);
  descriptor.set__type(ParameterType::PARAMETER_INTEGER_ARRAY);
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
void ArucoDetectorNode::declare_string_parameter(
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
SetParametersResult ArucoDetectorNode::on_set_parameters_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  // Initialize result object
  SetParametersResult res{};
  res.set__successful(true);
  res.set__reason("");

  // First, check if each update is feasible
  // Initial checks must be added here!
  for (const rclcpp::Parameter & p : params) {
    // Aruco side
    if (p.get_name() == "aruco_side") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for aruco_side");
        break;
      }
      continue;
    }

    // Camera offset
    if (p.get_name() == "camera_offset") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for camera_offset");
        break;
      }
      continue;
    }

    // Camera topic
    if (p.get_name() == "camera_topic") {
      if (p.get_type() != ParameterType::PARAMETER_STRING) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for camera_topic");
        break;
      }
      continue;
    }

    // Centering width
    if (p.get_name() == "centering_width") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for centering_width");
        break;
      }
      continue;
    }

    // Compute position flag
    if (p.get_name() == "compute_position") {
      if (p.get_type() != ParameterType::PARAMETER_BOOL) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for compute_position");
        break;
      }
      continue;
    }

    // Minimum error
    if (p.get_name() == "error_min") {
      if (p.get_type() != ParameterType::PARAMETER_DOUBLE) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for error_min");
        break;
      }
      continue;
    }

    // Rotate image flag
    if (p.get_name() == "rotate_image") {
      if (p.get_type() != ParameterType::PARAMETER_BOOL) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for rotate_image");
        break;
      }
      continue;
    }

    // Target IDs
    if (p.get_name() == "target_ids") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER_ARRAY) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for target_ids");
        break;
      }
      continue;
    }

    // Image transport
    if (p.get_name() == "transport") {
      if (p.get_type() != ParameterType::PARAMETER_STRING) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for transport");
        break;
      }
      continue;
    }

    // Focal length
    if (p.get_name() == "focal_length") {
      if (p.get_type() != ParameterType::PARAMETER_INTEGER) {
        res.set__successful(false);
        res.set__reason("Invalid parameter type for focal_length");
        break;
      }
      continue;
    }
  }
  if (!res.successful) {
    return res;
  }

  // Then, do what is necessary to update each parameter
  // Ad-hoc update procedures must be added here!
  for (const rclcpp::Parameter & p : params) {
    // Aruco side
    if (p.get_name() == "aruco_side") {
      aruco_side_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "aruco_side: %f m",
        aruco_side_);
      continue;
    }

    // Camera offset
    if (p.get_name() == "camera_offset") {
      camera_offset_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "camera_offset: %f m",
        camera_offset_);
      continue;
    }

    // Minimum error
    if (p.get_name() == "error_min") {
      error_min_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "error_min: %f m",
        error_min_);
      continue;
    }

    // Camera topic
    if (p.get_name() == "camera_topic") {
      camera_topic_ = p.as_string();
      RCLCPP_INFO(
        this->get_logger(),
        "camera_topic: %s",
        camera_topic_.c_str());
      continue;
    }

    // Centering width
    if (p.get_name() == "centering_width") {
      centering_width_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "centering_width: %ld px",
        centering_width_);
      continue;
    }

    // Compute position flag
    if (p.get_name() == "compute_position") {
      compute_position_ = p.as_bool();
      RCLCPP_INFO(
        this->get_logger(),
        "compute_position: %s",
        compute_position_ ? "true" : "false");
      continue;
    }

    // Rotate image flag
    if (p.get_name() == "rotate_image") {
      rotate_image_ = p.as_bool();
      RCLCPP_INFO(
        this->get_logger(),
        "rotate_image: %s",
        rotate_image_ ? "true" : "false");
      continue;
    }

    // Target IDs
    if (p.get_name() == "target_ids") {
      target_ids_ = p.as_integer_array();
      for (const auto & id : target_ids_) {
        RCLCPP_INFO(
          this->get_logger(),
          "target ID: %ld",
          id);
      }
      continue;
    }

    // Image transport
    if (p.get_name() == "transport") {
      transport_ = p.as_string();
      RCLCPP_INFO(
        this->get_logger(),
        "transport: %s",
        transport_.c_str());
      continue;
    }

    // Focal length
    if (p.get_name() == "focal_length") {
      focal_length_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "focal_length: %d px",
        focal_length_);
      continue;
    }
  }

  return res;
}

} // namespace ArucoDetector
