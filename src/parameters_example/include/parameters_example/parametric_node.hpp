/**
 * Parametric node structure.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * December 3, 2021
 */

#ifndef PARAMETRIC_PUB_HPP
#define PARAMETRIC_PUB_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

using namespace std_msgs::msg;

/**
 * Parametric publisher definition.
 */
class ParametricPub : public rclcpp::Node
{
public:
  ParametricPub();

private:
  int pub_num_; // Number to be published
  rcl_interfaces::msg::ParameterDescriptor param_descriptor_; // Number parameter descriptor

  rclcpp::Publisher<Int32>::SharedPtr num_publisher_;

  rclcpp::Subscription<Int32>::SharedPtr param_set_sub_;
  void param_set_clbk(const Int32::SharedPtr msg);

  //! There has to be, if needed, a single parameter set callback, with this syntax
  OnSetParametersCallbackHandle::SharedPtr param_clbk_handle_;
  rcl_interfaces::msg::SetParametersResult param_clbk(const std::vector<rclcpp::Parameter> &params);

  rclcpp::TimerBase::SharedPtr pub_timer_;
  void pub_routine();
};

#endif
