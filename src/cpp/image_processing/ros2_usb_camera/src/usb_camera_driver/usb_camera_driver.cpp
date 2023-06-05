/**
 * ROS 2 USB Camera Driver node implementation.
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

#include <usb_camera_driver/usb_camera_driver.hpp>

using namespace std::chrono_literals;
namespace USBCameraDriver
{

/**
 * @brief Builds a new CameraDriverNode.
 *
 * @param opts ROS 2 node options.
 *
 * @throws RuntimeError
 */
CameraDriverNode::CameraDriverNode(const rclcpp::NodeOptions & opts)
: Node("usb_camera_driver", opts)
{
  // Initialize node parameters
  init_parameters();

  // Initialize synchronization primitives
  stopped_.store(true, std::memory_order_release);

#ifdef WITH_CUDA
  // Check for GPU device availability
  if (!cv::cuda::getCudaEnabledDeviceCount()) {
    RCLCPP_FATAL(this->get_logger(), "No GPU device found");
    throw std::runtime_error("No GPU device found");
  }
  RCLCPP_INFO(this->get_logger(), "GPU device available");
#endif

  // Create and set up CameraInfoManager
  cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
  cinfo_manager_->setCameraName(this->get_parameter("camera_name").as_string());
  if (!cinfo_manager_->loadCameraInfo(this->get_parameter("camera_calibration_file").as_string())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get camera info");
  }

  // Create image_transport publishers (this will use all available transports, see docs)
  camera_pub_ = image_transport::create_camera_publisher(
    this,
    "~/" + this->get_parameter("base_topic_name").as_string() + "/image_color",
    this->get_parameter("best_effort_qos").as_bool() ?
    usb_camera_qos_profile : usb_camera_reliable_qos_profile);
  rect_pub_ = image_transport::create_publisher(
    this,
    "~/" + this->get_parameter("base_topic_name").as_string() + "/image_rect_color",
    this->get_parameter("best_effort_qos").as_bool() ?
    usb_camera_qos_profile : usb_camera_reliable_qos_profile);

  // Get and store current camera info and compute undistorsion and rectification maps
  if (cinfo_manager_->isCalibrated()) {
    camera_info_ = cinfo_manager_->getCameraInfo();
    A_ = cv::Mat(3, 3, CV_64FC1, camera_info_.k.data());
    D_ = cv::Mat(1, 5, CV_64FC1, camera_info_.d.data());
#ifdef WITH_CUDA
    cv::initUndistortRectifyMap(
      A_,
      D_,
      cv::Mat::eye(3, 3, CV_64F),
      A_,
      cv::Size(image_width_, image_height_),
      CV_32FC1,
      map1_,
      map2_);
    gpu_map1_.upload(map1_);
    gpu_map2_.upload(map2_);
#else
    cv::initUndistortRectifyMap(
      A_,
      D_,
      cv::Mat::eye(3, 3, CV_64F),
      A_,
      cv::Size(image_width_, image_height_),
      CV_16SC2,
      map1_,
      map2_);
#endif
  }

  // Initialize service servers
  hw_enable_server_ = this->create_service<SetBool>(
    "~/enable_camera",
    std::bind(
      &CameraDriverNode::hw_enable_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Cleans up stuff upon node termination.
 */
CameraDriverNode::~CameraDriverNode()
{
  // Stop camera sampling thread
  bool expected = false;
  if (stopped_.compare_exchange_strong(
      expected,
      true,
      std::memory_order_release,
      std::memory_order_acquire))
  {
    camera_sampling_thread_.join();
  }
  camera_pub_.shutdown();
  rect_pub_.shutdown();
}

/**
 * @brief Gets new frames from the camera and publishes them.
 */
void CameraDriverNode::camera_sampling_routine()
{
  // High-resolution sleep timer, in nanoseconds
  rclcpp::WallRate sampling_timer(std::chrono::nanoseconds(int(1.0 / double(fps_) * 1000000000.0)));

  RCLCPP_WARN(this->get_logger(), "Camera sampling thread started");

  while (true) {
    // Check if thread cancellation has been requested
    if (stopped_.load(std::memory_order_acquire)) {
      break;
    }

    // Get a new frame from the camera
    video_cap_ >> frame_;

    // Process the new frame
    if (!frame_.empty()) {
      rclcpp::Time timestamp = this->get_clock()->now();

      // Generate Image message
      Image::SharedPtr image_msg = nullptr, rect_image_msg = nullptr;
      if (is_flipped_) {
#ifdef WITH_CUDA
        gpu_frame_.upload(frame_);
        cv::cuda::flip(gpu_frame_, gpu_flipped_frame_, 0);
        gpu_flipped_frame_.download(flipped_frame_);
#else
        cv::flip(frame_, flipped_frame_, 0);
#endif
        if (cinfo_manager_->isCalibrated()) {
#ifdef WITH_CUDA
          cv::cuda::remap(
            gpu_flipped_frame_,
            gpu_rectified_frame_,
            gpu_map1_,
            gpu_map2_,
            cv::InterpolationFlags::INTER_LINEAR,
            cv::BorderTypes::BORDER_CONSTANT);
          gpu_rectified_frame_.download(rectified_frame_);
#else
          cv::remap(
            flipped_frame_,
            rectified_frame_,
            map1_,
            map2_,
            cv::InterpolationFlags::INTER_LINEAR,
            cv::BorderTypes::BORDER_CONSTANT);
#endif
          rect_image_msg = frame_to_msg(rectified_frame_);
          rect_image_msg->header.set__stamp(timestamp);
          rect_image_msg->header.set__frame_id(frame_id_);
        }
        image_msg = frame_to_msg(flipped_frame_);
      } else {
        if (cinfo_manager_->isCalibrated()) {
#ifdef WITH_CUDA
          gpu_frame_.upload(frame_);
          cv::cuda::remap(
            gpu_frame_,
            gpu_rectified_frame_,
            gpu_map1_,
            gpu_map2_,
            cv::InterpolationFlags::INTER_LINEAR,
            cv::BorderTypes::BORDER_CONSTANT);
          gpu_rectified_frame_.download(rectified_frame_);
#else
          cv::remap(
            frame_,
            rectified_frame_,
            map1_,
            map2_,
            cv::InterpolationFlags::INTER_LINEAR,
            cv::BorderTypes::BORDER_CONSTANT);
#endif
          rect_image_msg = frame_to_msg(rectified_frame_);
          rect_image_msg->header.set__stamp(timestamp);
          rect_image_msg->header.set__frame_id(frame_id_);
        }
        image_msg = frame_to_msg(frame_);
      }
      image_msg->header.set__stamp(timestamp);
      image_msg->header.set__frame_id(frame_id_);

      // Generate CameraInfo message
      CameraInfo::SharedPtr camera_info_msg = std::make_shared<CameraInfo>(camera_info_);
      camera_info_msg->header.set__stamp(timestamp);
      camera_info_msg->header.set__frame_id(frame_id_);

      // Publish new frame together with its CameraInfo on all available transports
      camera_pub_.publish(image_msg, camera_info_msg);
      if (cinfo_manager_->isCalibrated()) {
        rect_pub_.publish(rect_image_msg);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Empty frame");
    }

    sampling_timer.sleep();
  }

  // Close video capture device
  video_cap_.release();

  RCLCPP_WARN(this->get_logger(), "Camera sampling thread stopped");
}

/**
 * @brief Toggles the video capture device and related sampling thread.
 *
 * @param req Service request to parse.
 * @param resp Service response to populate.
 */
void CameraDriverNode::hw_enable_callback(
  SetBool::Request::SharedPtr req,
  SetBool::Response::SharedPtr resp)
{
  if (req->data) {
    bool expected = true;
    if (stopped_.compare_exchange_strong(
        expected,
        false,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      // Open capture device
      if (!video_cap_.open(this->get_parameter("camera_id").as_int(), cv::CAP_V4L2) ||
        !video_cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_) ||
        !video_cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_) ||
        !video_cap_.set(cv::CAP_PROP_FPS, fps_))
      {
        stopped_.store(true, std::memory_order_release);
        resp->set__success(false);
        resp->set__message("Failed to open capture device");
        RCLCPP_ERROR(this->get_logger(), "Failed to open capture device");
        return;
      }

      // Set camera parameters
      double exposure = this->get_parameter("exposure").as_double();
      double brightness = this->get_parameter("brightness").as_double();
      double wb_temperature = this->get_parameter("wb_temperature").as_double();
      bool success;
      if (exposure != 0.0) {
        success = video_cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);
        if (!success) {
          stopped_.store(true, std::memory_order_release);
          resp->set__success(false);
          resp->set__message("cv::VideoCapture::set(CAP_PROP_AUTO_EXPOSURE, 0.75) failed");
          RCLCPP_ERROR(this->get_logger(), "Failed to set camera exposure");
          return;
        }
        success = video_cap_.set(cv::CAP_PROP_EXPOSURE, exposure);
        if (!success) {
          stopped_.store(true, std::memory_order_release);
          resp->set__success(false);
          resp->set__message("cv::VideoCapture::set(CAP_PROP_EXPOSURE) failed");
          RCLCPP_ERROR(this->get_logger(), "Failed to set camera exposure");
          return;
        }
      }
      if (brightness != 0.0) {
        success = video_cap_.set(cv::CAP_PROP_BRIGHTNESS, brightness);
        if (!success) {
          stopped_.store(true, std::memory_order_release);
          resp->set__success(false);
          resp->set__message("cv::VideoCapture::set(CAP_PROP_BRIGHTNESS) failed");
          RCLCPP_ERROR(this->get_logger(), "Failed to set camera brightness");
          return;
        }
      }
      if (wb_temperature == 0.0) {
        success = video_cap_.set(cv::CAP_PROP_AUTO_WB, 1.0);
        if (!success) {
          stopped_.store(true, std::memory_order_release);
          resp->set__success(false);
          resp->set__message("cv::VideoCapture::set(CAP_PROP_AUTO_WB, 1.0) failed");
          RCLCPP_ERROR(this->get_logger(), "Failed to enable auto WB");
          return;
        }
      } else {
        success = video_cap_.set(cv::CAP_PROP_AUTO_WB, 0.0);
        if (!success) {
          stopped_.store(true, std::memory_order_release);
          resp->set__success(false);
          resp->set__message("cv::VideoCapture::set(CAP_PROP_AUTO_WB, 0.0) failed");
          RCLCPP_ERROR(this->get_logger(), "Failed to disable auto WB");
          return;
        }
        success = video_cap_.set(cv::CAP_PROP_WB_TEMPERATURE, wb_temperature);
        if (!success) {
          stopped_.store(true, std::memory_order_release);
          resp->set__success(false);
          resp->set__message("cv::VideoCapture::set(CAP_PROP_WB_TEMPERATURE) failed");
          RCLCPP_ERROR(this->get_logger(), "Failed to set WB temperature");
          return;
        }
      }

      // Start camera sampling thread
      camera_sampling_thread_ = std::thread{
        &CameraDriverNode::camera_sampling_routine,
        this};
    }
    resp->set__success(true);
    resp->set__message("");
  } else {
    bool expected = false;
    if (stopped_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      camera_sampling_thread_.join();
    }
    resp->set__success(true);
    resp->set__message("");
  }
}

} // namespace USBCameraDriver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(USBCameraDriver::CameraDriverNode)
