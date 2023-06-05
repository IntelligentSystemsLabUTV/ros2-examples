/**
 * Aruco Detector node implementation.
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

#include <string>
#include <stdexcept>

#include <aruco_detector/aruco_detector.hpp>

namespace ArucoDetector
{

/**
 * @brief Builds a new Aruco Detector node.
 *
 * @param opts Node options.
 *
 * @throws RuntimeError
 */
ArucoDetectorNode::ArucoDetectorNode(const rclcpp::NodeOptions & opts)
: Node("aruco_detector", opts)
{
  // Initialize synchronization primitives
  init_sync_primitives();

  // Initialize callback groups
  init_cgroups();

  // Initialize node parameters
  init_parameters();

  // Initialize topic publishers
  init_publishers();

  // Initialize topic subscriptions
  init_subscriptions();

  // Initialize services
  init_services();

  // Initialize camera ID for this Detector instance
  //std::string node_name(this->get_name());
  //if (node_name.find("bottom") != std::string::npos) {
  //  camera_id_ = Target::BOTTOM_CAMERA;
  //  RCLCPP_INFO(this->get_logger(), "Detector initializing for BOTTOM camera");
  //} else if (node_name.find("tilted") != std::string::npos) {
  //  camera_id_ = Target::TILTED_CAMERA;
  //  RCLCPP_INFO(this->get_logger(), "Detector initializing for TILTED camera");
  //} else if (node_name.find("front") != std::string::npos) {
  //  camera_id_ = Target::FRONT_CAMERA;
  //  RCLCPP_INFO(this->get_logger(), "Detector initializing for FRONT camera");
  //} else {
  //  camera_id_ = Target::UNKNOWN_CAMERA;
  //  RCLCPP_ERROR(this->get_logger(), "No specific camera set");
  //}

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Finalizes node operation.
 */
ArucoDetectorNode::~ArucoDetectorNode()
{
  // Unsubscribe from image topics
  if (is_on_) {
    camera_sub_.shutdown();
    is_on_ = false;
  }
  //target_img_pub_.shutdown();

  // Destroy synchronization primitives
  pthread_spin_destroy(&(this->pose_lock_));
}

/**
 * @brief Routine to initialize synchronization primitives.
 *
 * @throws RuntimeError
 */
void ArucoDetectorNode::init_sync_primitives()
{
  if (pthread_spin_init(&(this->pose_lock_), PTHREAD_PROCESS_PRIVATE)) {
    throw std::runtime_error("Failed to initialize spinlocks");
  }
}

/**
 * @brief Routine to initialize callback groups.
 */
void ArucoDetectorNode::init_cgroups()
{
  pose_cgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  enable_cgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Routine to initialize node parameters.
 */
void ArucoDetectorNode::init_parameters()
{
  // Register parameter updates callback
  on_set_params_chandle_ = this->add_on_set_parameters_callback(
    std::bind(
      &ArucoDetectorNode::on_set_parameters_callback,
      this,
      std::placeholders::_1));

  // Aruco side
  declare_double_parameter(
    "aruco_side",
    0.1, 0.1, 1.0, 0.0,
    "Aruco marker side length.",
    "Cannot be zero, must be in meters.",
    false,
    aruco_side_descriptor_);

  // Camera offset
  declare_double_parameter(
    "camera_offset",
    0.1, 0.0, 0.5, 0.0,
    "Distance between camera and drone center.",
    "Can be zero, must be in meters.",
    false,
    camera_offset_descriptor_);

  // Camera topic
  declare_string_parameter(
    "camera_topic",
    "/usb_camera_driver/camera/image_color",
    "Camera base topic name.",
    "Cannot be changed.",
    true,
    camera_topic_descriptor_);

  // Centering width
  declare_int_parameter(
    "centering_width",
    100, 2, 1000, 2,
    "Centering zone width [pixels].",
    "Cannot be changed.",
    true,
    centering_width_descriptor_);

  // Compute position flag
  declare_bool_parameter(
    "compute_position",
    false,
    "Enables target position computation.",
    "Cannot be changed.",
    true,
    compute_position_descriptor_);

  // Minimum error
  declare_double_parameter(
    "error_min",
    0.1, 0.0, 1.0, 0.0,
    "Minimum relevant displacement error. [m]",
    "Absolute, cannot be changed.",
    true,
    error_min_descriptor_);

  // Rotate image flag
  declare_bool_parameter(
    "rotate_image",
    false,
    "Enables rotation for vertically-mounted cameras.",
    "Cannot be changed.",
    true,
    rotate_image_descriptor_);

  // Target IDs
  declare_int_array_parameter(
    "target_ids",
    {15}, 0, 1023, 1,
    "IDs of targets to look for.",
    "Cannot be changed.",
    true,
    target_ids_descriptor_);

  // Image transport
  declare_string_parameter(
    "transport",
    "compressed",
    "Image transport to use.",
    "Cannot be changed.",
    true,
    transport_descriptor_);

  // Focal length
  declare_int_parameter(
    "focal_length",
    0, 0, 2047, 1,
    "Camera focal length.",
    "Cannot be changed.",
    true,
    focal_length_descriptor_);
}

/**
 * @brief Routine to initialize topic subscriptions.
 */
void ArucoDetectorNode::init_subscriptions()
{
  // Drone pose
  //auto pose_sub_opts = rclcpp::SubscriptionOptions();
  //pose_sub_opts.callback_group = pose_cgroup_;
  //pose_sub_ = this->create_subscription<Pose>(
  //  "/flight_control/pose",
  //  rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(FlightControl::fc_pose_qos_profile)),
  //  std::bind(
  //    &ArucoDetectorNode::pose_callback,
  //    this,
  //    std::placeholders::_1),
  //  pose_sub_opts);
}

/**
 * @brief Routine to initialize topic publishers.
 */
void ArucoDetectorNode::init_publishers()
{
  // Camera rate
  camera_rate_pub_ = this->create_publisher<Empty>(
    "~/camera_rate",
    rclcpp::QoS(1));

  // Target data
  //target_pub_ = this->create_publisher<Target>(
  //  "/targets",
  //  rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(ad_targets_qos_profile)));

  // Target images
  target_img_pub_ = image_transport::create_publisher(
    this,
    "~/targets/image_rect_color",
    rmw_qos_profile_sensor_data);
}

/**
 * @brief Routine to initialize service servers.
 */
void ArucoDetectorNode::init_services()
{
  // Enable
  enable_server_ = this->create_service<SetBool>(
    "~/enable",
    std::bind(
      &ArucoDetectorNode::enable_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    enable_cgroup_);
}

} // namespace ArucoDetector
