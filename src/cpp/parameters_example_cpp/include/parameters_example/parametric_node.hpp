/**
 * Parametric publisher definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * December 3, 2021
 */

#ifndef PARAMETRIC_PUB_HPP
#define PARAMETRIC_PUB_HPP

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int64.hpp>

using namespace std_msgs::msg;

/**
 * Node that publishes a number, which can be set by a parameter.
 */
class ParametricPub : public rclcpp::Node
{
public:
  ParametricPub();

private:
  int pub_num_; //! Number to be published
  rcl_interfaces::msg::ParameterDescriptor param_descriptor_; //! Number parameter descriptor

  rclcpp::Publisher<Int64>::SharedPtr num_publisher_;

  //! There has to be, if needed, a single parameter set callback, with this syntax
  OnSetParametersCallbackHandle::SharedPtr param_clbk_handle_;
  rcl_interfaces::msg::SetParametersResult param_clbk(const std::vector<rclcpp::Parameter> &params);

  rclcpp::TimerBase::SharedPtr pub_timer_;
  void pub_routine();
};

#endif
