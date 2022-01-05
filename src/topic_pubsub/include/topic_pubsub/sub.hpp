/**
 * Subscriber definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 22, 2021
 */

#ifndef SUB_HPP
#define SUB_HPP

#include <rclcpp/rclcpp.hpp> //! rclcpp base library

#ifndef EXAMPLE
#include <std_msgs/msg/string.hpp> //! interface library that we'll use
#else
#include <ros2_examples_interfaces/msg/string.hpp>
#endif

/**
 * Simple subscriber node: receives and prints strings transmitted on a topic.
 */
//! Every node must extend publicly the Node base class
class Sub : public rclcpp::Node
{
public:
  //! There must always be a constructor, with arbitrary input arguments
  Sub();

//! ROS-specific members better be private
private:
  //! DDS endpoint, acting as a subscriber
  //! When a message is received, a callback job is issued, which better be a private method
  //! Syntax is: rclcpp::Subscription<INTERFACE_TYPE>::SharedPtr OBJ;
  //! Callback signature must be:
  //!   void FUNC_NAME(const INTERFACE_TYPE::SharedPtr ARG_NAME);
#ifndef EXAMPLE
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  void msg_callback(const std_msgs::msg::String::SharedPtr msg);
#else
  rclcpp::Subscription<ros2_examples_interfaces::msg::String>::SharedPtr subscriber_;
  void msg_callback(const ros2_examples_interfaces::msg::String::SharedPtr msg);
#endif

  //! Nothing else is necessary to receive messages
};

#endif
