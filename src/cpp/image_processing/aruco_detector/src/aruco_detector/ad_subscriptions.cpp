/**
 * Aruco Detector node topic subscription callbacks.
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
 * @brief Searches targets in a new image.
 *
 * @param msg Image message to parse.
 */
void ArucoDetectorNode::camera_callback(const Image::ConstSharedPtr & msg)
{
  // Get current drone pose
  pthread_spin_lock(&(this->pose_lock_));
  DronePose current_pose = pose_;
  pthread_spin_unlock(&(this->pose_lock_));
  double altitude = current_pose.z;
  double yaw = current_pose.yaw;

  // Look for targets in the image
  cv::Mat new_frame(
    msg->height,
    msg->width,
    CV_8UC3,
    (void *)(msg->data.data()));

  // Detect targets
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(
    new_frame,
#ifdef ARUCO_API_OLD
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL),
#else
    std::make_shared<cv::aruco::Dictionary>(
      cv::aruco::getPredefinedDictionary(
        cv::aruco::PredefinedDictionaryType::DICT_ARUCO_ORIGINAL)),
#endif
    corners,
    ids);

  // Publish information about detected targets
  aruco_centers_.clear();
  for (int k = 0; k < int(ids.size()); k++) {
    if (std::find(target_ids_.begin(), target_ids_.end(), ids[k]) == target_ids_.end()) {
      continue;
    }

    //Target target_msg{};
    //target_msg.set__camera(camera_id_);
    //target_msg.set__id(ids[k]);

    // Detect target center
    double x1 = corners[k][0].x;
    double y1 = corners[k][0].y;

    double x2 = corners[k][1].x;
    double y2 = corners[k][1].y;

    double x3 = corners[k][2].x;
    double y3 = corners[k][2].y;

    double x4 = corners[k][3].x;
    double y4 = corners[k][3].y;

    double xc_den = (-((x2 - x4) * (y1 - y3)) + (x1 - x3) * (y2 - y4));
    double yc_den = (-((x2 - x4) * (y1 - y3)) + (x1 - x3) * (y2 - y4));

    if ((abs(xc_den) < 1e-5) || (abs(yc_den) < 1e-5)) {
      // Do not divide by zero! Just discard this sample
      continue;
    }

    int xc =
      (x3 * x4 * (y1 - y2) + x1 * x4 * (y2 - y3) + x1 * x2 * (y3 - y4) + x2 * x3 * (-y1 + y4)) /
      xc_den;
    int yc =
      (x4 * y2 * (y1 - y3) + x1 * y2 * y3 - x2 * y1 * y4 - x1 * y3 * y4 + x2 * y3 * y4 + x3 * y1 *
      (-y2 + y4)) / yc_den;

    //if (camera_id_ == Target::BOTTOM_CAMERA && ids[k] == 1) {
    //  // If precise centering is required (e.g. for landing), apply camera offset
    //  xc += sqrt((pow(x1 - x3, 2) + pow(y1 - y3, 2)) / 2) * camera_offset_ / aruco_side_;
    //}

    aruco_centers_.push_back(cv::Point(xc, yc));

    // Detect in which side of the image the target lies
    if (rotate_image_) {
      if (yc < (new_frame.size().height / 2) - (centering_width_ / 2)) {
        //target_msg.set__centering(Target::LEFT);
      } else if (yc > (new_frame.size().height / 2) + (centering_width_ / 2)) {
        //target_msg.set__centering(Target::RIGHT);
      } else {
        //target_msg.set__centering(Target::CENTER);
      }
    } else {
      if (xc < (new_frame.size().width / 2) - (centering_width_ / 2)) {
        //target_msg.set__centering(Target::LEFT);
      } else if (xc > (new_frame.size().width / 2) + (centering_width_ / 2)) {
        //target_msg.set__centering(Target::RIGHT);
      } else {
        //target_msg.set__centering(Target::CENTER);
      }
    }

    // Compute target position w.r.t. the world NED reference frame
    if (compute_position_) {
      double ex_pixels = xc - (new_frame.size().width / 2);
      double ey_pixels = yc - (new_frame.size().height / 2);

      double err_x = -(altitude * ex_pixels) / focal_length_;
      double err_y = -(altitude * ey_pixels) / focal_length_;

      // If error is too low, zero it to avoid numerical noise
      if (sqrt(pow(err_x, 2) + pow(err_y, 2)) <= error_min_) {
        err_x = 0.0;
        err_y = 0.0;
      }

      double cy = cos(yaw);
      double sy = sin(yaw);

      double x_err = err_x * cy - err_y * sy;
      double y_err = err_x * sy + err_y * cy;

      // Round errors to centimeters to cut out numerical noise
      x_err = round_space(x_err, 100.0f);
      y_err = round_space(y_err, 100.0f);

      float new_x = current_pose.x + x_err;
      float new_y = current_pose.y + y_err;

      //target_msg.set__position({new_x, new_y});

    } else {
      //target_msg.set__position({NAN, NAN});
    }

    //target_pub_->publish(target_msg);
  }

  // Draw search output, ROI and HUD in another image
  cv::aruco::drawDetectedMarkers(new_frame, corners, ids);
  for (auto center : aruco_centers_) {
    cv::drawMarker(
      new_frame,
      center,
      cv::Scalar(0, 255, 0),
      cv::MARKER_DIAMOND,
      20,
      3);
  }
  camera_frame_ = new_frame; // Doesn't copy image data, but sets data type...
  if (rotate_image_) {
    // ... Which must be properly set here
    cv::rotate(new_frame, camera_frame_, cv::ROTATE_90_COUNTERCLOCKWISE);
  }
  cv::Point line_left_top(
    (camera_frame_.size().width / 2) - (centering_width_ / 2),
    0);
  cv::Point line_right_top(
    (camera_frame_.size().width / 2) + (centering_width_ / 2),
    0);
  cv::Point line_left_bottom(
    (camera_frame_.size().width / 2) - (centering_width_ / 2),
    camera_frame_.size().height - 1);
  cv::Point line_right_bottom(
    (camera_frame_.size().width / 2) + (centering_width_ / 2),
    camera_frame_.size().height - 1);
  cv::Point rect_p1(
    (camera_frame_.size().width / 2) - (centering_width_ / 2),
    (camera_frame_.size().height / 2) - (centering_width_ / 2));
  cv::Point rect_p2(
    (camera_frame_.size().width / 2) + (centering_width_ / 2),
    (camera_frame_.size().height / 2) + (centering_width_ / 2));
  cv::Point crosshair_p(
    camera_frame_.size().width / 2,
    camera_frame_.size().height / 2);
  cv::line(
    camera_frame_,
    line_left_top,
    line_left_bottom,
    cv::Scalar(0, 255, 0),
    5);
  cv::line(
    camera_frame_,
    line_right_top,
    line_right_bottom,
    cv::Scalar(0, 255, 0),
    5);
  cv::rectangle(
    camera_frame_,
    rect_p1,
    rect_p2,
    cv::Scalar(0, 255, 0),
    5);
  cv::drawMarker(
    camera_frame_,
    crosshair_p,
    cv::Scalar(0, 255, 0),
    cv::MARKER_CROSS,
    15,
    3);

  // Publish rate message and processed image
  Empty rate_msg{};
  camera_rate_pub_->publish(rate_msg);
  Image::SharedPtr processed_image_msg = frame_to_msg(camera_frame_);
  processed_image_msg->header.set__stamp(this->get_clock()->now());
  processed_image_msg->header.set__frame_id("map");
  target_img_pub_.publish(processed_image_msg);
}

/**
 * @brief Stores the latest drone pose.
 *
 * @param msg Pose message to parse.
 */
/*void ArucoDetectorNode::pose_callback(const Pose::SharedPtr msg)
{
  DronePose new_pose(
    msg->x,
    msg->y,
    msg->z,
    msg->roll,
    msg->pitch,
    msg->yaw);
  pthread_spin_lock(&(this->pose_lock_));
  pose_ = new_pose;
  pthread_spin_unlock(&(this->pose_lock_));
}*/

} // namespace ArucoDetector
