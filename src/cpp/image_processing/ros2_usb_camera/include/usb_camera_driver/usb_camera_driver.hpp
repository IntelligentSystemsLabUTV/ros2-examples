/**
 * ROS 2 USB Camera Driver node.
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

#ifndef ROS2_USB_CAMERA_USB_CAMERA_DRIVER_HPP
#define ROS2_USB_CAMERA_USB_CAMERA_DRIVER_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <camera_info_manager/camera_info_manager.hpp>

#include <image_transport/image_transport.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#ifdef WITH_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include <rmw/types.h>

using namespace rcl_interfaces::msg;
using namespace sensor_msgs::msg;
using namespace std_srvs::srv;

namespace USBCameraDriver
{

/**
 * QoS profile for best-effort image transmission.
 */
static const rmw_qos_profile_t usb_camera_qos_profile = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  1,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

/**
 * QoS profile for image transmission.
 */
static const rmw_qos_profile_t usb_camera_reliable_qos_profile = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  1,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

/**
 * Drives USB, V4L-compatible cameras with OpenCV.
 */
class CameraDriverNode : public rclcpp::Node
{
public:
  explicit CameraDriverNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  ~CameraDriverNode();

private:
  /* Video capture device and buffers */
  cv::VideoCapture video_cap_;
  cv::Mat frame_;
  cv::Mat flipped_frame_;
  cv::Mat rectified_frame_;
  cv::Mat A_, D_;
  cv::Mat map1_, map2_;

#ifdef WITH_CUDA
  cv::cuda::GpuMat gpu_frame_;
  cv::cuda::GpuMat gpu_flipped_frame_;
  cv::cuda::GpuMat gpu_rectified_frame_;
  cv::cuda::GpuMat gpu_map1_, gpu_map2_;
#endif

  /* Node parameters */
  std::string frame_id_;
  int64_t fps_ = 0;
  int64_t image_height_ = 0;
  int64_t image_width_ = 0;
  bool is_flipped_ = false;

  /* Service servers */
  rclcpp::Service<SetBool>::SharedPtr hw_enable_server_;

  /* Service callbacks */
  void hw_enable_callback(SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr resp);

  /* Node parameters descriptors */
  ParameterDescriptor base_topic_name_descriptor_;
  ParameterDescriptor be_qos_descriptor_;
  ParameterDescriptor camera_calibration_file_descriptor_;
  ParameterDescriptor camera_id_descriptor_;
  ParameterDescriptor camera_name_descriptor_;
  ParameterDescriptor exposure_descriptor_;
  ParameterDescriptor frame_id_descriptor_;
  ParameterDescriptor fps_descriptor_;
  ParameterDescriptor image_height_descriptor_;
  ParameterDescriptor image_width_descriptor_;
  ParameterDescriptor is_flipped_descriptor_;
  ParameterDescriptor wb_temperature_descriptor_;

  /* image_transport objects */
  image_transport::CameraPublisher camera_pub_;
  image_transport::Publisher rect_pub_;
  camera_info_manager::CameraInfo camera_info_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;

  /* Utility routines */
  Image::SharedPtr frame_to_msg(cv::Mat & frame);
  void declare_bool_parameter(
    std::string && name,
    bool default_val,
    std::string && desc, std::string && constraints,
    bool read_only, ParameterDescriptor & descriptor);
  void declare_double_parameter(
    std::string && name,
    double default_val, double from, double to, double step,
    std::string && desc, std::string && constraints,
    bool read_only, ParameterDescriptor & descriptor);
  void declare_int_parameter(
    std::string && name,
    int64_t default_val, int64_t from, int64_t to, int64_t step,
    std::string && desc, std::string && constraints,
    bool read_only, ParameterDescriptor & descriptor);
  void declare_string_parameter(
    std::string && name,
    std::string && default_val,
    std::string && desc, std::string && constraints,
    bool read_only, ParameterDescriptor & descriptor);
  void init_parameters();

  /* Parameters callback */
  OnSetParametersCallbackHandle::SharedPtr on_set_params_chandle_;
  SetParametersResult on_set_parameters_callback(
    const std::vector<rclcpp::Parameter> & params);

  /* Thread objects and routines */
  std::thread camera_sampling_thread_;
  void camera_sampling_routine();

  /* Synchronization primitives */
  std::atomic<bool> stopped_;
};

} // namespace USBCameraDriver

#endif // ROS2_USB_CAMERA_USB_CAMERA_DRIVER_HPP
