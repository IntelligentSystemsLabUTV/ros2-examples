/**
 * Aruco Detector node definition.
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

#ifndef STANIS_ARUCO_DETECTOR_HPP
#define STANIS_ARUCO_DETECTOR_HPP

#include <memory>
#include <opencv2/aruco.hpp>
#ifdef ARUCO_API_OLD
#include <opencv2/aruco/dictionary.hpp>
#endif
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pthread.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
//#include <stanis_interfaces/msg/pose.hpp>
//#include <stanis_interfaces/msg/target.hpp>
#include <std_msgs/msg/empty.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>

//#include <stanis_qos/aruco_detector_qos.hpp>
//#include <stanis_qos/flight_control_qos.hpp>

using namespace rcl_interfaces::msg;
using namespace sensor_msgs::msg;
//using namespace stanis_interfaces::msg;
using namespace std_msgs::msg;
using namespace std_srvs::srv;

namespace ArucoDetector
{

/**
 * Drone pose.
 */
struct DronePose
{
  float x = 0.0f; // m
  float y = 0.0f; // m
  float z = 0.0f; // m

  float roll = 0.0f;  // rad, [-PI +PI]
  float pitch = 0.0f; // rad, [-PI +PI]
  float yaw = 0.0f;   // rad, [-PI +PI]

  DronePose() {}

  DronePose(
    float pose_x, float pose_y, float pose_z,
    float pose_roll, float pose_pitch, float pose_yaw)
  : x(pose_x), y(pose_y), z(pose_z),
    roll(pose_roll), pitch(pose_pitch), yaw(pose_yaw)
  {}
};

/**
 * Main target detection node.
 */
class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  virtual ~ArucoDetectorNode();

private:
  /* Node initialization routines */
  void init_sync_primitives();
  void init_cgroups();
  void init_parameters();
  void init_subscriptions();
  void init_publishers();
  void init_services();

  /* Topic subscriptions callback groups */
  rclcpp::CallbackGroup::SharedPtr pose_cgroup_;

  /* Topic subscriptions */
  //rclcpp::Subscription<Pose>::SharedPtr pose_sub_;

  /* image_transport subscriptions */
  image_transport::Subscriber camera_sub_;

  /* Topic subscriptions callbacks */
  void camera_callback(const Image::ConstSharedPtr & msg);
  //void pose_callback(const Pose::SharedPtr msg);

  /* Topic publishers */
  rclcpp::Publisher<Empty>::SharedPtr camera_rate_pub_;
  //rclcpp::Publisher<Target>::SharedPtr target_pub_;

  /* image_transport publishers */
  image_transport::Publisher target_img_pub_;

  /* Service servers callback groups */
  rclcpp::CallbackGroup::SharedPtr enable_cgroup_;

  /* Service servers */
  rclcpp::Service<SetBool>::SharedPtr enable_server_;

  /* Service callbacks */
  void enable_callback(
    SetBool::Request::SharedPtr req,
    SetBool::Response::SharedPtr resp);

  /* Data buffers */
  cv::Mat camera_frame_;
  std::vector<cv::Point> aruco_centers_;

  /* Internal state variables */
  uint8_t camera_id_ = 0;
  bool is_on_ = false;
  DronePose pose_{};

  /* Node parameters */
  double aruco_side_ = 0.0;
  double camera_offset_ = 0.0;
  std::string camera_topic_ = "";
  int64_t centering_width_ = 0;
  bool compute_position_ = false;
  double error_min_ = 0.0;
  int focal_length_ = 0;
  bool rotate_image_ = false;
  std::vector<int64_t> target_ids_ = {};
  std::string transport_ = "";

  /* Node parameters descriptors */
  ParameterDescriptor aruco_side_descriptor_;
  ParameterDescriptor camera_offset_descriptor_;
  ParameterDescriptor camera_topic_descriptor_;
  ParameterDescriptor centering_width_descriptor_;
  ParameterDescriptor compute_position_descriptor_;
  ParameterDescriptor error_min_descriptor_;
  ParameterDescriptor rotate_image_descriptor_;
  ParameterDescriptor target_ids_descriptor_;
  ParameterDescriptor transport_descriptor_;
  ParameterDescriptor focal_length_descriptor_;

  /* Synchronization primitives */
  pthread_spinlock_t pose_lock_;

  /* Parameters callback */
  OnSetParametersCallbackHandle::SharedPtr on_set_params_chandle_;
  SetParametersResult on_set_parameters_callback(
    const std::vector<rclcpp::Parameter> & params);

  /* Utility routines */
  Image::SharedPtr frame_to_msg(cv::Mat & frame);
  float round_angle(float num, float prec);
  float round_space(float num, float prec);
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
  void declare_int_array_parameter(
    std::string && name,
    std::vector<int64_t> && default_val, int64_t from, int64_t to, int64_t step,
    std::string && desc, std::string && constraints,
    bool read_only, ParameterDescriptor & descriptor);
  void declare_string_parameter(
    std::string && name,
    std::string && default_val,
    std::string && desc, std::string && constraints,
    bool read_only, ParameterDescriptor & descriptor);
};

} // namespace ArucoDetector

#endif // STANIS_ARUCO_DETECTOR_HPP
