/**
 * Aruco Detector node service callbacks.
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
 * @brief Toggles target detection.
 *
 * @param req Service request to parse.
 * @param rest Service response to populate.
 */
void ArucoDetectorNode::enable_callback(
  SetBool::Request::SharedPtr req,
  SetBool::Response::SharedPtr resp)
{
  if (req->data) {
    if (!is_on_) {
      camera_sub_ = image_transport::create_subscription(
        this,
        camera_topic_,
        std::bind(
          &ArucoDetectorNode::camera_callback,
          this,
          std::placeholders::_1),
        transport_,
        rmw_qos_profile_sensor_data);
      is_on_ = true;
      RCLCPP_WARN(this->get_logger(), "Detector ACTIVATED");
    }
    resp->set__success(true);
    resp->set__message("");
  } else {
    if (is_on_) {
      camera_sub_.shutdown();
      is_on_ = false;
      RCLCPP_WARN(this->get_logger(), "Detector DEACTIVATED");
    }
    resp->set__success(true);
    resp->set__message("");
  }
}

} // namespace ArucoDetector
